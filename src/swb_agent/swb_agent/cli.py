"""`swb-agent`: check an agent, start a new one (roadmap 030)."""

import argparse
import logging
import sys

from . import load_config


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv[:1] == ["supervise"]:
        from .supervise import main as supervise
        return supervise(argv[1:])
    parser = argparse.ArgumentParser(prog="swb-agent")
    sub = parser.add_subparsers(dest="command", required=True)
    check = sub.add_parser("check", help="conformance against a fake relay, no simulator")
    check.add_argument("agent", help="module:Class or path/to/file.py:Class")
    check.add_argument("--config", help="JSON or YAML passed to the agent's constructor")
    check.add_argument("--reset-timeout", type=float, default=30.0)
    check.add_argument("-v", "--verbose", action="store_true", help="show the SDK's log")
    sub.add_parser("supervise", help="run an autopilot and its agent; restart them on request "
                   "(swb-agent supervise -h)")
    new = sub.add_parser("new", help="a project skeleton for an agent")
    new.add_argument("name")
    new.add_argument("--dir", default=".")
    args = parser.parse_args(argv)

    if args.command == "check":
        from .check import load_class, run_check
        logging.basicConfig(level=logging.INFO if args.verbose else logging.CRITICAL,
                            format="%(asctime)s %(levelname)s %(name)s: %(message)s")
        agent_class = load_class(args.agent)
        config = load_config(args.config) if args.config else {}
        print(f"swb-agent check {args.agent}")
        report = run_check(agent_class, config, args.reset_timeout)
        print(report.text())
        return 0 if report.passed else 1
    if args.command == "new":
        from .skeleton import create
        root, spec = create(args.name, args.dir)
        print(f"created {root}; check it with: cd {root} && swb-agent check {spec}")
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
