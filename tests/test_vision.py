import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import unittest
import cv2
import numpy as np
from so101_gomoku.vision.board_detector import VisionConfig, IntersectionDetector, BoardObserver
from so101_gomoku.contracts.models import Cell
from so101_gomoku.game.board import GameState
from so101_gomoku.orchestration.controller import GameController, Phase
from so101_gomoku.orchestration.simulation import SimulatedTable
from so101_gomoku.ai.alpha_beta import AlphaBetaEngine


def fixture(size=9):
    cfg = VisionConfig(intersections=size)
    empty = np.full((cfg.image_size, cfg.image_size, 3), (110, 170, 210), np.uint8)
    for i in range(size):
        cv2.line(empty, cfg.point(i, 0), cfg.point(i, size-1), (45, 45, 45), 2)
        cv2.line(empty, cfg.point(0, i), cfg.point(size-1, i), (45, 45, 45), 2)
    corners = [cfg.point(0, 0), cfg.point(0, size-1), cfg.point(size-1, size-1), cfg.point(size-1, 0)]
    return cfg, empty, IntersectionDetector(empty, corners, 'test-cal', cfg)


class VisionTests(unittest.TestCase):
    def test_empty_and_intersection_stones_all_edges(self):
        cfg, empty, detector = fixture()
        self.assertTrue(all(c == Cell.EMPTY for row in detector.detect(empty, frame_id='0', captured_at=1).observation.cells for c in row))
        image = empty.copy()
        stones = [(0,0,Cell.BLACK), (0,8,Cell.WHITE), (8,0,Cell.WHITE), (8,8,Cell.BLACK), (4,4,Cell.BLACK), (4,5,Cell.WHITE)]
        for r,c,color in stones:
            cv2.circle(image, cfg.point(r,c), 20, (20,20,20) if color == Cell.BLACK else (245,245,245), -1)
        result = detector.detect(image, frame_id='1', captured_at=2)
        for r,c,color in stones:
            self.assertEqual(result.observation.cells[r][c], color, (r,c,result.reasons[r][c]))
        self.assertEqual(result.observation.cells[3][3], Cell.EMPTY)

    def test_square_center_is_not_a_move(self):
        cfg, empty, detector = fixture()
        image = empty.copy()
        x,y=cfg.point(3,3)
        cv2.circle(image,(x+32,y+32),20,(20,20,20),-1)
        obs=detector.detect(image,frame_id='1',captured_at=1).observation
        self.assertFalse(any(c == Cell.BLACK for row in obs.cells for c in row))

    def test_large_occlusion_and_explicit_mask(self):
        cfg,empty,detector=fixture()
        image=empty.copy()
        x,y=cfg.point(3,3)
        cv2.rectangle(image,(x-35,y-30),(x+110,y+90),(80,100,180),-1)
        obs=detector.detect(image,frame_id='1',captured_at=1).observation
        self.assertEqual(obs.cells[3][3],Cell.UNKNOWN)
        self.assertEqual(obs.cells[0][0],Cell.EMPTY)
        mask=np.zeros(empty.shape[:2],np.uint8)
        cv2.circle(mask,cfg.point(0,0),20,255,-1)
        self.assertEqual(detector.detect(empty,frame_id='2',captured_at=2,occlusion_mask=mask).observation.cells[0][0],Cell.UNKNOWN)

    def test_ten_intersections_and_bad_image(self):
        cfg,empty,detector=fixture(10)
        self.assertEqual(len(detector.detect(empty,frame_id='1',captured_at=1).observation.cells),10)
        with self.assertRaises(ValueError):
            detector.detect(empty[:100],frame_id='2',captured_at=2)
        for margin in (48.,float('nan')):
            with self.assertRaises(ValueError): VisionConfig(margin_px=margin)

    def test_perspective_rectification_and_light_change_rejection(self):
        cfg,empty,_=fixture()
        image=empty.copy()
        cv2.circle(image,cfg.point(2,6),20,(245,245,245),-1)
        source=np.float32([[0,0],[608,0],[608,608],[0,608]])
        destination=np.float32([[30,20],[620,0],[640,620],[0,640]])
        h=cv2.getPerspectiveTransform(source,destination)
        corners=np.float32([cfg.point(0,0),cfg.point(0,8),cfg.point(8,8),cfg.point(8,0)])
        warped_corners=cv2.perspectiveTransform(corners[None],h)[0]
        ref=cv2.warpPerspective(empty,h,(660,660))
        detector=IntersectionDetector(ref,warped_corners,'perspective',cfg)
        current=cv2.warpPerspective(image,h,(660,660))
        result=detector.detect(current,frame_id='1',captured_at=1)
        self.assertEqual(result.observation.cells[2][6],Cell.WHITE)
        bright=cv2.convertScaleAbs(ref,alpha=1,beta=60)
        result=detector.detect(bright,frame_id='2',captured_at=2)
        self.assertTrue(any(c==Cell.UNKNOWN for row in result.observation.cells for c in row))

    def test_default_robot_is_black_and_observer_bridge(self):
        cfg,empty,detector=fixture()
        class Camera:
            def capture(self): return empty, 1.
        self.assertEqual(BoardObserver(Camera(),detector).observe().cells, GameState.empty(9).cells)
        table=SimulatedTable()
        c=GameController(table,AlphaBetaEngine(max_depth=1),table,table.calibration_id,clock=table.clock)
        self.assertEqual(c.start(),Phase.THINKING)
        self.assertEqual(c.run_turn(),Phase.WAIT_HUMAN)
        self.assertEqual(c.state.last_move.color,Cell.BLACK)


if __name__=='__main__': unittest.main()
