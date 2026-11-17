"""Use upstream Koch NN weights and TreeNode with project freestyle GameState.

MIT upstream in vendor/koch; see THIRD_PARTY_NOTICES.md. We do not instantiate
upstream VanillaBoard or call its hardware-masked/random get_action path.
"""
from pathlib import Path
import time
import numpy as np
from so101_gomoku.contracts.models import Cell, Move
from .common import Decision, tactical_move, empty_points, validate_search
from .candidates import candidates
from .vendor.koch.mcts_alphaZero import TreeNode


def encode_board(state):
    """Preserve upstream five-channel encoding and row flip, with no robot mask."""
    if state.size != 9:
        raise ValueError('Koch pretrained model requires 9x9 board')
    planes = np.zeros((5, 9, 9), dtype=np.float32)
    cells = np.asarray(state.cells)
    planes[0] = cells == state.next_player
    planes[1] = cells == -state.next_player
    if state.last_move is not None:
        r, c = state.last_move.row, state.last_move.col
        if not (0 <= r < 9 and 0 <= c < 9) or cells[r, c] != state.last_move.color:
            raise ValueError('Inconsistent last move for network input')
        planes[2, r, c] = 1
    if np.count_nonzero(cells) % 2 == 0:
        planes[3] = 1
    # Old hardware-restricted channel intentionally zero: all legal cells allowed.
    return np.ascontiguousarray(planes[:, ::-1, :])


class KochMCTSEngine:
    def __init__(self, model_path, n_playouts=64, c_puct=5.):
        import torch
        from .vendor.koch.policy_value_net import VanillaPolicyValueNet
        if type(n_playouts) is not int or n_playouts < 1 or not np.isfinite(c_puct) or c_puct <= 0:
            raise ValueError('Positive playout count and exploration coefficient required')
        self.model_path = Path(model_path).resolve(strict=True)
        self.model = VanillaPolicyValueNet(9, 9).cpu()
        weights = torch.load(self.model_path, map_location='cpu', weights_only=True)
        self.model.load_state_dict(weights, strict=True)
        self.model.eval()
        self.n_playouts, self.c_puct = n_playouts, c_puct

    def policy_value(self, state):
        import torch
        with torch.inference_mode():
            policy, value = self.model(torch.from_numpy(encode_board(state)).unsqueeze(0))
        legal = empty_points(state)
        flat = policy.numpy().reshape(-1)
        probabilities = np.array([flat[r*9+c] for r, c in legal], float)
        total = probabilities.sum()
        if not np.isfinite(probabilities).all() or not np.isfinite(value.item()) or total <= 0:
            raise RuntimeError('Invalid pretrained network output')
        return list(zip(legal, probabilities / total)), float(value.item())

    def choose_move(self, state, time_limit_s=.5):
        validate_search(state, time_limit_s)
        if state.size != 9:
            raise ValueError('Koch pretrained model requires 9x9 board')
        start = time.monotonic()
        tactical = tactical_move(state)
        if tactical:
            return Decision(tactical[0], 'koch-mcts', tactical[1], elapsed_s=time.monotonic()-start)
        root = TreeNode(None, 1.)
        count = 0
        for _ in range(self.n_playouts):
            if count and time.monotonic()-start >= time_limit_s:
                break
            board, node = state, root
            while not node.is_leaf():
                point, node = node.select(self.c_puct)
                board = board.play(Move(*point, board.next_player))
            # Upstream MCTS sequence adapted: terminal first, no invalid policy call.
            winner = board.winner()
            if board.finished:
                value = 0. if winner is None else (1. if winner == board.next_player else -1.)
            else:
                priors, value = self.policy_value(board)
                node.expand(priors, forbidden_moves=[], is_you_black=False)
            node.update_recursive(-value)
            count += 1
        if not root._children:
            raise RuntimeError('MCTS did not expand any legal root action')
        # Deterministic action; no training noise, random sampling or hardware masks.
        point, child = max(root._children.items(), key=lambda item: (item[1]._n_visits, item[1]._P, -item[0][0], -item[0][1]))
        return Decision(Move(*point, state.next_player), 'koch-mcts', 'pretrained_mcts', child._Q,
                        nodes=count, elapsed_s=time.monotonic()-start)
