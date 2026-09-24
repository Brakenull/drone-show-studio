"""python -m tools.studio_bridge <command> ...  (run from the repo root; docs/5-studio_gui.md §4)."""

from __future__ import annotations

import argparse
import importlib
import platform
import sys
import traceback
from pathlib import Path

from .events import EXIT_INPUT, EXIT_INTERNAL, EXIT_OK, emit, log
from .paths import REPO_ROOT, find_extension_dir, find_packer


def cmd_doctor(_: argparse.Namespace) -> int:
    checks = []

    def check(name: str, required_for: str, fn) -> None:
        try:
            detail = fn()
            checks.append({"name": name, "ok": True, "detail": detail, "required_for": required_for})
        except Exception as exc:  # report every missing piece, never stop at the first
            checks.append({"name": name, "ok": False, "detail": str(exc), "required_for": required_for})

    def module_version(module: str):
        return lambda: getattr(importlib.import_module(module), "__version__", "installed")

    def drone_core_check():
        from .paths import import_drone_core

        dc = import_drone_core()
        if not hasattr(dc, "SafetyViolationError"):
            raise ImportError(f"{dc.__file__} predates SafetyViolationError; rebuild stage2_core_engine")
        return dc.__file__

    def jsonschema_check():
        from importlib.metadata import version

        importlib.import_module("jsonschema")
        return version("jsonschema")

    def packer_check():
        packer = find_packer()
        if packer is None:
            raise FileNotFoundError("pack_to_binary not built (stage3_simulation_packer/build)")
        return str(packer)

    check("python", "all", lambda: f"{platform.python_version()} ({sys.executable})")
    check("drone_core", "stage2", drone_core_check)
    check("numpy", "all", module_version("numpy"))
    check("scipy", "replay", module_version("scipy"))
    check("jsonschema", "validate", jsonschema_check)
    check("pyarrow", "stage2 (Arrow output)", module_version("pyarrow"))
    check("warp", "stage3", module_version("warp"))
    check("pack_to_binary", "stage3", packer_check)
    emit("doctor", repo_root=str(REPO_ROOT), extension_dir=str(find_extension_dir() or ""), checks=checks)
    emit("done", status="succeeded", exit_code=EXIT_OK)
    return EXIT_OK


def _validation_event(result: dict) -> None:
    emit("validation", ok=result["ok"], errors=result["errors"], warnings=result["warnings"],
         summary=result.get("summary"))


def cmd_validate(args: argparse.Namespace) -> int:
    from .validate import validate_file

    result = validate_file(Path(args.phase1_json))
    _validation_event(result)
    code = EXIT_OK if result["ok"] else EXIT_INPUT
    emit("done", status="succeeded" if result["ok"] else "failed_input", exit_code=code)
    return code


def cmd_new_run(args: argparse.Namespace) -> int:
    from .runs import create_run
    from .validate import validate_file

    source = Path(args.phase1_json).resolve()
    result = validate_file(source)
    _validation_event(result)
    if not result["ok"]:
        emit("done", status="failed_input", exit_code=EXIT_INPUT)
        return EXIT_INPUT
    run_dir = create_run(source, Path(args.runs_dir).resolve(), result["data"])
    emit("run_created", run_id=run_dir.name, run_dir=str(run_dir))
    emit("done", status="succeeded", exit_code=EXIT_OK)
    return EXIT_OK


def cmd_stage2(args: argparse.Namespace) -> int:
    from .stage2_job import run_stage2

    return run_stage2(Path(args.run_dir), Path(args.overrides) if args.overrides else None)


def cmd_replay(args: argparse.Namespace) -> int:
    from .stage2_job import rebuild_replay

    return rebuild_replay(Path(args.run_dir))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="studio_bridge", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("doctor").set_defaults(fn=cmd_doctor)
    p = sub.add_parser("validate")
    p.add_argument("phase1_json")
    p.set_defaults(fn=cmd_validate)
    p = sub.add_parser("new-run")
    p.add_argument("phase1_json")
    p.add_argument("--runs-dir", required=True)
    p.set_defaults(fn=cmd_new_run)
    p = sub.add_parser("stage2")
    p.add_argument("run_dir")
    p.add_argument("--overrides", default=None, help="JSON file passed as optional_config_overrides")
    p.set_defaults(fn=cmd_stage2)
    p = sub.add_parser("replay")
    p.add_argument("run_dir")
    p.set_defaults(fn=cmd_replay)
    args = parser.parse_args(argv)
    try:
        return args.fn(args)
    except Exception as exc:
        log(traceback.format_exc())
        emit("error", code="internal", message=f"{type(exc).__name__}: {exc}")
        emit("done", status="failed_error", exit_code=EXIT_INTERNAL)
        return EXIT_INTERNAL


if __name__ == "__main__":
    sys.exit(main())
