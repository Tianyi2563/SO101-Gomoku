"""Recognize a photo using manually calibrated grid intersections and empty photo."""
import argparse
import json
from dataclasses import asdict
from pathlib import Path
import sys
import time
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
from so101_gomoku.vision.board_detector import VisionConfig, IntersectionDetector


def read_image(path):
    image=cv2.imdecode(np.fromfile(path,dtype=np.uint8),cv2.IMREAD_COLOR)
    if image is None: raise ValueError(f'Cannot read image: {path}')
    return image


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--image',type=Path,required=True)
    p.add_argument('--output',type=Path,default=ROOT/'data/vision-result')
    args=p.parse_args()
    config=json.loads(args.config.read_text(encoding='utf-8-sig'))
    ref=Path(config['empty_image'])
    if not ref.is_absolute(): ref=args.config.parent/ref
    detector=IntersectionDetector(read_image(ref),config['outer_intersections_xy'],config['calibration_id'],VisionConfig(**config.get('vision',{})))
    result=detector.detect(read_image(args.image),frame_id=args.image.name,captured_at=time.time())
    args.output.mkdir(parents=True,exist_ok=True)
    payload={'mode':'OFFLINE_IMAGE','observation':asdict(result.observation),'reasons':result.reasons,
             'notes':['Rule scores, not calibrated probabilities','Image processing time is not original photo capture time','Corners denote outer intersections, not board edges']}
    (args.output/'board.json').write_text(json.dumps(payload,indent=2),encoding='utf-8')
    for name,img in [('rectified',result.rectified),('overlay',result.overlay),('occlusion',result.occlusion_mask)]:
        ok,encoded=cv2.imencode('.png',img)
        if not ok: raise RuntimeError('PNG encoding failed')
        encoded.tofile(args.output/f'{name}.png')
    print(np.array(result.observation.cells))
    print('0=empty, 1=black, -1=white, 2=unknown')


if __name__=='__main__': main()
