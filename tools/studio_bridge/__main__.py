"""python -m tools.studio_bridge <command> ...  (run from the repo root; docs/5-studio_gui.md §4)."""

from __future__ import annotations

import argparse
import importlib
import platform
import sys
import traceback
from pathlib import Path

from .events import EXIT_INPUT, EXIT_INTERNAL, EXIT_OK, emit, log, reserve_stdout
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
    check("pyopencl", "stage3", module_version("pyopencl"))
    check("pack_to_binary", "stage3", packer_check)

    devices: list[dict] = []

    def opencl_check():
        from .stage3_job import simulation_devices

        devices.extend(simulation_devices())
        if not devices:
            raise RuntimeError("no OpenCL device: install a GPU driver or a CPU OpenCL runtime")
        return ", ".join(f"{d['name']} ({d['id']})" for d in devices)

    check("opencl", "stage3", opencl_check)
    # `devices`: ids for --device, the default ("auto": first GPU) first; `device_info` describes each id.
    emit("doctor", repo_root=str(REPO_ROOT), extension_dir=str(find_extension_dir() or ""), checks=checks,
         devices=["auto", *(d["id"] for d in devices)], device_info=devices)
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
    overrides = None
    if args.overrides_json is not None:
        import json

        from .config_fields import override_errors

        overrides = json.loads(args.overrides_json)
        errors = override_errors(overrides)
        if errors:
            emit("error", code="input", message="Planner settings rejected: " + "; ".join(errors))
            emit("done", status="failed_input", exit_code=EXIT_INPUT)
            return EXIT_INPUT
    run_dir = create_run(source, Path(args.runs_dir).resolve(), result["data"],
                         copy_from=Path(args.copy_from) if args.copy_from else None, overrides=overrides)
    emit("run_created", run_id=run_dir.name, run_dir=str(run_dir))
    emit("done", status="succeeded", exit_code=EXIT_OK)
    return EXIT_OK


def cmd_stage2(args: argparse.Namespace) -> int:
    import json

    from .stage2_job import run_stage2

    if args.overrides_json is not None:
        overrides = json.loads(args.overrides_json)
    elif args.overrides:
        overrides = json.loads(Path(args.overrides).read_text(encoding="utf-8"))
    else:
        overrides = {}
    return run_stage2(Path(args.run_dir), overrides)


def _formation_list(text: str) -> list[int]:
    try:
        return [int(part) for part in text.split(",") if part.strip()]
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected formation indices like 0,2,3, got {text!r}") from None


def _formation_duration(text: str) -> tuple[int, float]:
    try:
        k, seconds = text.split("=", 1)
        return int(k), float(seconds)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected FORMATION=SECONDS like 2=30, got {text!r}") from None


def cmd_stage2_returns(args: argparse.Namespace) -> int:
    from .returns_job import run_stage2_returns

    return run_stage2_returns(Path(args.run_dir), args.formations, dict(args.duration_s or []))


def cmd_config(args: argparse.Namespace) -> int:
    """The planner settings editor's data for a run (docs/5-studio_gui.md §6.2)."""
    import json

    from . import config_fields

    run_dir = Path(args.run_dir)
    meta = json.loads((run_dir / "input" / "phase1.json").read_text(encoding="utf-8"))["project_metadata"]
    saved = run_dir / "stage2" / "config_overrides.json"
    overrides = json.loads(saved.read_text(encoding="utf-8")) if saved.exists() else {}
    emit("config", **config_fields.describe(meta, overrides))
    emit("done", status="succeeded", exit_code=EXIT_OK)
    return EXIT_OK


def cmd_replay(args: argparse.Namespace) -> int:
    if args.return_from is not None:
        from .returns_job import rebuild_return_replay

        return rebuild_return_replay(Path(args.run_dir), args.return_from)
    from .stage2_job import rebuild_replay

    return rebuild_replay(Path(args.run_dir))


def cmd_monte_carlo(args: argparse.Namespace) -> int:
    from .stage3_job import run_monte_carlo_job

    return run_monte_carlo_job(Path(args.run_dir), args.runs, args.device, args.batch, args.seed)


def cmd_pack(args: argparse.Namespace) -> int:
    from .stage3_job import run_pack_job

    return run_pack_job(Path(args.run_dir))


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
    p.add_argument("--copy-from", default=None,
                   help="Run folder this one copies (its input is phase1_json); keeps its planner settings")
    p.add_argument("--overrides-json", default=None, help="Planner settings for the new run (instead of the copied ones)")
    p.set_defaults(fn=cmd_new_run)
    p = sub.add_parser("stage2")
    p.add_argument("run_dir")
    p.add_argument("--overrides", default=None, help="JSON file passed as optional_config_overrides")
    p.add_argument("--overrides-json", default=None, help="The same as a JSON string (what the Studio sends)")
    p.set_defaults(fn=cmd_stage2)
    p = sub.add_parser("stage2-returns")
    p.add_argument("run_dir")
    p.add_argument("--formations", type=_formation_list, default=None,
                   help="Formation indices to plan returns from, e.g. 0,2 (default: every formation, except "
                        "the last when the show has a return leg)")
    p.add_argument("--duration-s", type=_formation_duration, action="append", metavar="K=SECONDS",
                   help="Target duration of formation K's return (default: Auto, the minimum); repeatable")
    p.set_defaults(fn=cmd_stage2_returns)
    p = sub.add_parser("config")
    p.add_argument("run_dir")
    p.set_defaults(fn=cmd_config)
    p = sub.add_parser("replay")
    p.add_argument("run_dir")
    p.add_argument("--return", dest="return_from", type=int, default=None, metavar="K",
                   help="Rebuild the replay of formation K's return path instead of the show's")
    p.set_defaults(fn=cmd_replay)
    p = sub.add_parser("monte_carlo")
    p.add_argument("run_dir")
    p.add_argument("--runs", type=int, default=100)
    p.add_argument("--device", default="auto", help="OpenCL device: auto (first GPU), gpu, cpu or opencl:P:D")
    p.add_argument("--batch", type=int, default=0, help="Runs simulated together (default: automatic)")
    # Sent by Studio builds that predate the OpenCL twin (one process per CPU core); runs now batch on
    # the device instead, so this is accepted and ignored.
    p.add_argument("--workers", type=int, default=None, help=argparse.SUPPRESS)
    p.add_argument("--seed", type=int, default=None, help="Base seed (default: the runner's)")
    p.set_defaults(fn=cmd_monte_carlo)
    p = sub.add_parser("pack")
    p.add_argument("run_dir")
    p.set_defaults(fn=cmd_pack)
    args = parser.parse_args(argv)
    reserve_stdout()
    try:
        return args.fn(args)
    except Exception as exc:
        log(traceback.format_exc())
        emit("error", code="internal", message=f"{type(exc).__name__}: {exc}")
        emit("done", status="failed_error", exit_code=EXIT_INTERNAL)
        return EXIT_INTERNAL


if __name__ == "__main__":
    sys.exit(main())
