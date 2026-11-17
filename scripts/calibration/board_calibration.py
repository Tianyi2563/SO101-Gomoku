"""Manual calibration CLI; no robot or camera connection."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
from so101_gomoku.calibration.workflow import point_template,set_point,fit_dataset,save_calibration,load_calibration
from so101_gomoku.recording.storage import atomic_json,read_json


def main():
    p=argparse.ArgumentParser(description=__doc__)
    commands=p.add_subparsers(dest='command',required=True)
    init=commands.add_parser('init');init.add_argument('--file',type=Path,required=True)
    init.add_argument('--size',type=int,default=10);init.add_argument('--spacing-mm',type=float,default=21.)
    init.add_argument('--source',choices=['manual_input','fk_model','synthetic'],default='manual_input')
    add=commands.add_parser('set-point');add.add_argument('--file',type=Path,required=True)
    add.add_argument('--row',type=int,required=True);add.add_argument('--col',type=int,required=True)
    add.add_argument('--xyz-mm',type=float,nargs=3,required=True);add.add_argument('--role',choices=['fit','validation'],required=True)
    fit=commands.add_parser('fit');fit.add_argument('--file',type=Path,required=True);fit.add_argument('--output',type=Path,required=True)
    fit.add_argument('--limit-mm',type=float,default=3.);fit.add_argument('--id')
    check=commands.add_parser('check');check.add_argument('--file',type=Path,required=True)
    args=p.parse_args()
    if args.command=='init':
        if args.file.exists(): raise FileExistsError('Refusing to overwrite existing point dataset')
        atomic_json(args.file,point_template(args.size,args.spacing_mm,args.source))
        print('Template saved; fill every point before fitting. Spacing remains unverified.')
    elif args.command=='set-point':
        atomic_json(args.file,set_point(read_json(args.file),args.row,args.col,args.xyz_mm,args.role))
        print('Point saved (internal units: meters)')
    else:
        if args.command=='fit':
            record=fit_dataset(read_json(args.file),args.limit_mm,args.id)
            save_calibration(args.output,record)
            record=load_calibration(args.output)
        else: record=load_calibration(args.file)
        print(f"calibration_id={record['calibration_id']}; source={record['measurement_source']}")
        for role in ('fit','validation'):
            print(role,record['report'][role])
        print('within_validation_limit:',record['within_validation_limit'],'; hardware accuracy NOT established')
        if not record['within_validation_limit']:raise SystemExit(2)


if __name__=='__main__':main()
