"""Compatibility entry point; shares the workbench/CLI reset implementation."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description='单次上半身 reseterror()；不会使能。先查明故障原因。默认只显示用法。')
    p.add_argument('--execute-reset',action='store_true');p.add_argument('--user',default='fairino');a=p.parse_args()
    if not a.execute_reset:p.print_help();return 0
    sys.path.insert(0, str(ROOT))
    from scripts.pico4.fairino.clear_errors import main as clear_errors
    return clear_errors(['--part', 'upper', '--execute', '--user', a.user])
if __name__=='__main__':raise SystemExit(main())
