"""Real AI + controller, synthetic camera/robot. --interactive for terminal play."""
from pathlib import Path
import sys
import argparse
import json
from dataclasses import asdict

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from so101_gomoku.ai.alpha_beta import AlphaBetaEngine
from so101_gomoku.contracts.models import Cell, Move
from so101_gomoku.game.board import GameState
from so101_gomoku.orchestration.controller import GameController, Phase
from so101_gomoku.orchestration.simulation import SimulatedTable


def print_board(state):
    chars = {Cell.EMPTY: '.', Cell.BLACK: 'X', Cell.WHITE: 'O'}
    print('  ' + ' '.join(map(str, range(state.size))))
    for r, row in enumerate(state.cells):
        print(str(r) + ' ' + ' '.join(chars[c] for c in row))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--engine', choices=['alphabeta', 'koch-mcts'], default='alphabeta')
    parser.add_argument('--rounds', type=int, default=5)
    parser.add_argument('--size', type=int, default=10, help='Number of intersections per side (9 squares = 10)')
    parser.add_argument('--interactive', action='store_true')
    parser.add_argument('--human', choices=['black', 'white'], default='white')
    parser.add_argument('--time-limit', type=float, default=.2)
    parser.add_argument('--playouts', type=int, default=64)
    parser.add_argument('--fail-on-round', type=int, default=0)
    parser.add_argument('--output', type=Path, help='Optional simulation report path')
    args = parser.parse_args()
    if args.rounds < 1:
        parser.error('--rounds must be >=1')
    if args.size < 5:
        parser.error('--size must be >=5 intersections')
    if args.engine == 'koch-mcts' and args.size != 9:
        parser.error('Koch pretrained model supports only --size 9; use alphabeta for the actual 10x10 board')
    if args.engine == 'koch-mcts':
        import torch
        torch.set_num_threads(1)
        from so101_gomoku.ai.koch_mcts import KochMCTSEngine
        engine = KochMCTSEngine(ROOT/'assets/models/koch/Vanilla_MCTS.model', n_playouts=args.playouts)
    else:
        engine = AlphaBetaEngine()
    opponent = AlphaBetaEngine(max_depth=2)
    initial_state = GameState.empty(args.size)
    table = SimulatedTable(initial_state)
    color = Cell.BLACK if args.human == 'black' else Cell.WHITE
    controller = GameController(table, engine, table, table.calibration_id,
                                state=initial_state, human_color=color, search_time_s=args.time_limit, clock=table.clock)
    controller.start()
    completed = 0
    print('OFFLINE SIMULATION: real AI, synthetic observations and robot. No hardware.')
    while controller.phase not in (Phase.PAUSED, Phase.FINISHED):
        if not args.interactive and completed >= args.rounds:
            break
        if controller.phase == Phase.WAIT_HUMAN:
            if args.interactive:
                print_board(controller.state)
                text = input(f'Your move: row col (0..{args.size-1}), or q: ').strip()
                if text.lower() == 'q':
                    break
                try:
                    r, c = map(int, text.split())
                    table.human_move(Move(r, c, color))
                except (ValueError, TypeError) as exc:
                    print(f'Invalid move: {exc}')
                    continue
            else:
                table.human_move(opponent.choose_move(table.board, args.time_limit).move)
        table.fail_execution = args.fail_on_round == completed+1
        previous_decision = controller.last_decision
        controller.run_turn()
        completed += 1
        if controller.last_decision and controller.last_decision is not previous_decision:
            d = controller.last_decision
            print(f'{d.engine}: ({d.move.row},{d.move.col}), {d.reason}; phase={controller.phase.value}')
    report = {'mode': 'OFFLINE_SIMULATION', 'hardware_connected': False, 'engine': args.engine,
              'human_input': 'terminal' if args.interactive else 'alphabeta_opponent',
              'round_calls': completed, 'phase': controller.phase.value, 'pause_reason': controller.pause_reason,
              'game_version': controller.state.version, 'board': controller.state.cells,
              'physical_board_matches_committed': controller.state.cells == table.board.cells,
              'robot_tasks': [asdict(task) for task in table.tasks], 'events': controller.events,
              'limitations': ['Synthetic camera, grasp, robot and physical board', 'No AI strength benchmark']}
    path = args.output or ROOT / f'data/sessions/gomoku_{args.engine.replace("-", "_")}_simulation.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print_board(controller.state)
    print(f'phase={controller.phase.value}, version={controller.state.version}, robot_tasks={len(table.tasks)}')
    if controller.pause_reason:
        print(controller.pause_reason)
    if controller.phase == Phase.PAUSED and not args.fail_on_round:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
