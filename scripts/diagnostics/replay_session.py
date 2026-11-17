"""Replay journal states/events without importing or calling any robot driver."""
from pathlib import Path
import argparse
import json
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
from so101_gomoku.recording.session import replay_session,SessionJournal
from so101_gomoku.recording.storage import atomic_json


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('session',type=Path)
    p.add_argument('--event',type=int,help='Show board snapshot at sequence number')
    p.add_argument('--export',type=Path,help='Export full JSON timeline')
    p.add_argument('--html',type=Path,help='Export self-contained read-only viewer with available images')
    p.add_argument('--repair-tail',action='store_true',help='Explicitly back up and drop incomplete final record')
    args=p.parse_args()
    if args.repair_tail:
        with SessionJournal(args.session,repair_tail=True):pass
    result=replay_session(args.session)
    if args.export:atomic_json(args.export,result)
    if args.html:
        from so101_gomoku.recording.replay_html import export_replay_html
        export_replay_html(args.session,args.html)
    events=[r for r in result['records'] if r['kind']=='event']
    print('records:',len(result['records']),'events:',len(events),'version:',result['checkpoint']['state']['version'])
    for record in events:
        print(record['sequence'],record['event']['event'],record['checkpoint']['phase'],record['checkpoint']['state']['version'])
    if args.event is not None:
        record=result['records'][args.event]
        if record['kind']=='event':
            chars={0:'.',1:'X',-1:'O'}
            for row in record['checkpoint']['state']['cells']:print(' '.join(chars[c] for c in row))
        print(json.dumps(record,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
