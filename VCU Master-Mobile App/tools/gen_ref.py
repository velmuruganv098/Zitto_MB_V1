"""Generate reference data from the web app's own Python code for the Android port.

Run with the desktop venv:  "VCU Master/files/VCU_Master/vcu_master/.venv/Scripts/python.exe" tools/gen_ref.py
(first copy VCU Master/files/VCU_Master/vcu_master/dbc_library to app/src/main/assets/dbc_library).

Outputs:
  <app>/src/main/assets/library_index.json   {rel: [[id, ext], ...]}   (cantools frame ids)
  <app>/src/test/resources/analyzer_ref.json {rel: roles_meta-like dict incl. binding factor/offset}
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
WEB = HERE.parents[1] / "VCU Master" / "files" / "VCU_Master" / "vcu_master"
APP = HERE.parent / "app"
sys.path.insert(0, str(WEB))

import cantools  # noqa: E402
from vcu_master.dbc_engine import DbcEngine  # noqa: E402

lib = WEB / "dbc_library"
index, ref = {}, {}
for p in sorted(lib.rglob("*.dbc")):
    rel = p.relative_to(lib).as_posix()
    text = p.read_text(errors="replace")
    try:
        db = cantools.database.load_string(text, database_format="dbc", strict=False)
        index[rel] = [[m.frame_id, bool(m.is_extended_frame)] for m in db.messages]
    except Exception as exc:  # noqa: BLE001
        print("skip", rel, exc)
        index[rel] = []
        continue
    eng = DbcEngine()
    try:
        eng.load(p.name, text, [1, 2])
    except Exception as exc:  # noqa: BLE001
        print("engine skip", rel, exc)
        continue
    meta = eng.roles_meta()
    a = eng.analyses.get(p.name)
    roles = {}
    for k, r in meta["roles"].items():
        role = a.roles[k]
        roles[k] = {
            "label": r["label"], "unit": r["unit"], "kind": r["kind"], "panel": r["panel"],
            "section": r["section"], "min": r["min"], "max": r["max"], "step": r["step"],
            "index": r["index"], "derived": r["derived"],
            "choices": {str(c): v for c, v in (r["choices"] or {}).items()},
            "bindings": [[b.msg, b.sig, b.factor, b.offset] for b in role.bindings],
        }
    ref[rel] = {
        "roles": roles, "cells": meta["cells"], "temps": meta["temps"], "balance": meta["balance"],
        "panels": [x["key"] for x in meta["panels"]],
        "flags": {k: [[f["msg"], f["sig"], f["kind"]] for f in v] for k, v in meta["flags"].items()},
        "msg_system": a.msg_system,
    }

(APP / "src/main/assets").mkdir(parents=True, exist_ok=True)
(APP / "src/main/assets/library_index.json").write_text(json.dumps(index, separators=(",", ":")))
(APP / "src/test/resources/analyzer_ref.json").write_text(json.dumps(ref, indent=0))
print(len(index), "indexed,", len(ref), "analyzed")
