import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import unittest
from dataclasses import replace
from so101_gomoku.contracts.models import Cell, Move, ExecutionResult
from so101_gomoku.game.board import GameState
from so101_gomoku.ai.alpha_beta import AlphaBetaEngine
from so101_gomoku.orchestration.controller import GameController, Phase
from so101_gomoku.orchestration.simulation import SimulatedTable
from so101_gomoku.orchestration.executor import TaskExecutor


class ControllerTests(unittest.TestCase):
    def make(self, state=None, human_color=Cell.BLACK):
        # Deterministically increasing epoch-like clock shared by observations/controller.
        self.t = 1000.
        def clock():
            self.t += .001
            return self.t
        table = SimulatedTable(state, clock=clock)
        controller = GameController(table, AlphaBetaEngine(max_depth=1), table, table.calibration_id,
                                    state=state, human_color=human_color, clock=clock, search_time_s=.1)
        self.assertNotEqual(controller.start(), Phase.PAUSED)
        return table, controller

    def test_human_robot_round_and_commit_after_verify(self):
        table, c = self.make()
        table.human_move(Move(4, 4, Cell.BLACK))
        original_execute = table.execute
        def execute(task):
            self.assertEqual(c.state.version, 1)
            self.assertEqual(c.state.cells[task.move.row][task.move.col], Cell.EMPTY)
            self.assertEqual(c.phase, Phase.EXECUTING)
            return original_execute(task)
        table.execute = execute
        self.assertEqual(c.run_turn(), Phase.WAIT_HUMAN)
        self.assertEqual(c.state.version, 2)
        self.assertEqual(c.state.cells, table.board.cells)
        self.assertIsNone(c.pending_task)

    def test_no_human_move_is_wait_not_failure(self):
        table, c = self.make()
        self.assertEqual(c.run_turn(), Phase.WAIT_HUMAN)
        self.assertEqual(len(table.tasks), 0)

    def test_execution_failure_pauses_no_retry_and_explicit_recovery(self):
        table, c = self.make()
        table.human_move(Move(4, 4, Cell.BLACK))
        table.fail_execution = True
        self.assertEqual(c.run_turn(), Phase.PAUSED)
        self.assertEqual(c.state.version, 1)
        self.assertIsNotNone(c.pending_task)
        with self.assertRaises(RuntimeError):
            c.run_turn()
        with self.assertRaises(RuntimeError):
            c.recover()
        self.assertEqual(len(table.tasks), 1)
        self.assertEqual(c.recover(robot_stopped=True), Phase.THINKING)
        self.assertEqual(len(table.tasks), 1)

    def test_uncertain_execution_completed_can_be_reconciled(self):
        table, c = self.make()
        table.human_move(Move(4, 4, Cell.BLACK))
        table.fail_execution = table.apply_despite_failure = True
        self.assertEqual(c.run_turn(), Phase.PAUSED)
        self.assertEqual(c.state.version, 1)
        self.assertEqual(c.recover(robot_stopped=True), Phase.WAIT_HUMAN)
        self.assertEqual(c.state.version, 2)
        self.assertEqual(len(table.tasks), 1)

    def test_success_without_piece_does_not_commit(self):
        table, c = self.make()
        table.human_move(Move(4, 4, Cell.BLACK))
        def fake_success(task):
            return ExecutionResult(task.task_id, True, 'false_positive', visual_confirmed=True)
        table.execute = fake_success
        self.assertEqual(c.run_turn(), Phase.PAUSED)
        self.assertEqual(c.state.version, 1)
        self.assertIsNotNone(c.pending_task)

    def test_illegal_multi_change_and_stale_calibration(self):
        table, c = self.make()
        table.human_move(Move(4, 4, Cell.BLACK))
        table.human_move(Move(3, 3, Cell.WHITE))
        self.assertEqual(c.run_turn(), Phase.PAUSED)
        self.assertEqual(c.state.version, 0)
        self.assertEqual(len(table.tasks), 0)
        table, c = self.make()
        table.calibration_id = 'changed'
        self.assertEqual(c.run_turn(), Phase.PAUSED)

    def test_unknown_and_stale_frames(self):
        table, c = self.make()
        original = table.observe
        cells = [list(row) for row in table.board.cells]
        cells[0][0] = Cell.UNKNOWN
        table.observe = lambda: replace(original(), cells=tuple(map(tuple, cells)))
        self.assertEqual(c.run_turn(), Phase.PAUSED)
        self.assertEqual(len(table.tasks), 0)
        table, c = self.make()
        original = table.observe
        table.observe = lambda: replace(original(), captured_at=1.)
        self.assertEqual(c.run_turn(), Phase.PAUSED)

    def test_robot_first_and_nonreentrant(self):
        table, c = self.make(human_color=Cell.WHITE)
        self.assertEqual(c.phase, Phase.THINKING)
        self.assertEqual(c.run_turn(), Phase.WAIT_HUMAN)
        self.assertEqual(c.state.version, 1)
        c._lock.acquire()
        try:
            with self.assertRaises(RuntimeError):
                c.run_turn()
        finally:
            c._lock.release()

    def test_human_win_prevents_robot_execution(self):
        cells = [[Cell.EMPTY]*9 for _ in range(9)]
        cells[0][:4] = [Cell.BLACK]*4
        table, c = self.make(GameState(tuple(map(tuple, cells))))
        table.human_move(Move(0, 4, Cell.BLACK))
        self.assertEqual(c.run_turn(), Phase.FINISHED)
        self.assertEqual(len(table.tasks), 0)

    def test_ai_error_and_wrong_task_id_pause(self):
        table, c = self.make()
        table.human_move(Move(4, 4, Cell.BLACK))
        c.engine.choose_move = lambda *_: (_ for _ in ()).throw(RuntimeError('AI failed'))
        self.assertEqual(c.run_turn(), Phase.PAUSED)
        self.assertEqual(len(table.tasks), 0)
        table, c = self.make()
        table.human_move(Move(4, 4, Cell.BLACK))
        table.execute = lambda task: ExecutionResult('wrong-id', True, 'bad')
        self.assertEqual(c.run_turn(), Phase.PAUSED)
        self.assertEqual(c.state.version, 1)

    def test_board_changed_during_thinking_blocks_execution(self):
        table, c = self.make()
        table.human_move(Move(4, 4, Cell.BLACK))
        choose = c.engine.choose_move
        def change_board(state, budget):
            d = choose(state, budget)
            table.board = table.board.play(Move(0, 0, Cell.WHITE))
            return d
        c.engine.choose_move = change_board
        self.assertEqual(c.run_turn(), Phase.PAUSED)
        self.assertEqual(c.state.version, 1)
        self.assertEqual(table.tasks, [])

    def test_frames_buffered_during_ai_cannot_authorize_action(self):
        table, c = self.make()
        table.human_move(Move(4, 4, Cell.BLACK))
        choose, live_observe = c.engine.choose_move, table.observe
        def buffer_old_board(state, budget):
            decision = choose(state, budget)
            queued = iter([live_observe(), live_observe()])
            table.board = table.board.play(Move(0, 0, Cell.WHITE))
            table.observe = lambda: next(queued)
            return decision
        c.engine.choose_move = buffer_old_board
        self.assertEqual(c.run_turn(), Phase.PAUSED)
        self.assertEqual(table.tasks, [])
        self.assertEqual(c.state.version, 1)

    def test_recovery_requires_frames_after_recovery_start(self):
        table, c = self.make()
        table.human_move(Move(4, 4, Cell.BLACK))
        table.fail_execution = True
        c.run_turn()
        buffered = iter([table.observe(), table.observe()])
        table.observe = lambda: next(buffered)
        self.assertEqual(c.recover(robot_stopped=True), Phase.PAUSED)
        self.assertEqual(c.state.version, 1)

    def test_recovery_wrong_board_and_repeated_frame_rejected(self):
        table, c = self.make()
        table.human_move(Move(4, 4, Cell.BLACK))
        table.fail_execution = True
        c.run_turn()
        table.board = table.board.play(Move(8, 8, Cell.WHITE))
        self.assertEqual(c.recover(robot_stopped=True), Phase.PAUSED)
        self.assertEqual(c.state.version, 1)
        table, c = self.make()
        obs = table.observe()
        table.observe = lambda: obs
        self.assertEqual(c.run_turn(), Phase.PAUSED)

    def test_robot_win_ends_game_after_verification(self):
        cells = [[Cell.EMPTY]*9 for _ in range(9)]
        cells[0][:4] = [Cell.WHITE]*4
        table, c = self.make(GameState(tuple(map(tuple, cells)), next_player=Cell.WHITE))
        self.assertEqual(c.run_turn(), Phase.FINISHED)
        self.assertEqual(c.state.winner(), Cell.WHITE)
        self.assertEqual(len(table.tasks), 1)

    def test_task_executor_bridge(self):
        class Primitive:
            def execute(self, task_id, plan):
                return ExecutionResult(task_id, True, str(plan))
        from so101_gomoku.contracts.models import RobotTask
        task = RobotTask('t', Move(0, 0, Cell.BLACK), 0, 'cal')
        executor = TaskExecutor(lambda task: 'plan', Primitive(), 'cal')
        self.assertTrue(executor.execute(task).completed)
        self.assertFalse(executor.execute(replace(task, calibration_id='wrong')).completed)
        executor.planner = lambda *_: (_ for _ in ()).throw(ValueError('unreachable'))
        self.assertFalse(executor.execute(task).completed)


if __name__ == '__main__':
    unittest.main()
