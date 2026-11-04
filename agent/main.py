import argparse
from agent.loop import run_once

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=1234)
    parser.add_argument('--command', type=str, default='Bring the round red thing')
    parser.add_argument('--dummy', action='store_true')
    args = parser.parse_args()
    res = run_once(args.seed, args.command, dummy=args.dummy)
    print(res)

if __name__ == '__main__':
    main()
