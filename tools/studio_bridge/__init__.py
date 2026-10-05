"""Python side of the Studio desktop app.

The Tauri shell starts `python -m tools.studio_bridge <command> ...` from the
repo root and reads NDJSON events from stdout. All pipeline work goes through
the existing entry points (drone_core, stage3 loaders); nothing here
re-implements solver or evaluator logic.
"""
