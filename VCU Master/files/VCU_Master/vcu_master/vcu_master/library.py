"""
library.py - The DBC library shipped with VCU Master (same tree as CAN_DBC_Simulator's
dbc_library/<Product>/<Vendor>/*.dbc) and automatic DBC matching.

The bridge forwards every CAN frame, but frames are only decoded once a DBC that
contains their identifier is loaded.  This module indexes the identifiers of every
library DBC so that the frames actually seen on the bus can be matched to the DBC
that describes them (e.g. a Daly BMS on 0x18904001..0x18984001).
"""

from __future__ import annotations

import json
import logging
import threading
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

import cantools

log = logging.getLogger("vcu_master.library")

Key = Tuple[int, bool]          # (frame id, extended)


class DbcLibrary:
    def __init__(self, root: Path, cache: Path) -> None:
        self.root = root
        self.cache = cache
        self.items: List[Dict[str, Any]] = []      # [{id, product, vendor, file, ids: set[Key]}]
        self._lock = threading.Lock()
        self._ready = False

    # ------------------------------------------------------------ index
    def ensure(self) -> None:
        with self._lock:
            if self._ready:
                return
            self._build()
            self._ready = True

    def _build(self) -> None:
        if not self.root.is_dir():
            log.warning("DBC library folder %s not found", self.root)
            return
        try:
            cached = json.loads(self.cache.read_text()) if self.cache.exists() else {}
        except Exception:                                   # noqa: BLE001
            cached = {}
        out, fresh = [], {}
        for p in sorted(self.root.rglob("*.dbc")):
            rel = p.relative_to(self.root).as_posix()
            st = p.stat()
            stamp = f"{st.st_size}:{int(st.st_mtime)}"
            c = cached.get(rel)
            if c and c.get("stamp") == stamp:
                ids = c["ids"]
            else:
                try:
                    db = cantools.database.load_file(str(p), strict=False)
                    ids = [[m.frame_id, bool(m.is_extended_frame)] for m in db.messages]
                except Exception as exc:                    # noqa: BLE001
                    log.info("library: skip %s (%s)", rel, exc)
                    ids = []
            fresh[rel] = {"stamp": stamp, "ids": ids}
            parts = rel.split("/")
            out.append({
                "id": rel,
                "product": parts[0] if len(parts) > 1 else "",
                "vendor": parts[1] if len(parts) > 2 else "",
                "file": parts[-1],
                "ids": {(int(i), bool(e)) for i, e in ids},
            })
        self.items = out
        try:
            self.cache.write_text(json.dumps(fresh))
        except Exception:                                   # noqa: BLE001
            pass

    # ------------------------------------------------------------ queries
    def list(self) -> List[Dict[str, Any]]:
        self.ensure()
        return [{"id": i["id"], "product": i["product"], "vendor": i["vendor"], "file": i["file"],
                 "messages": len(i["ids"])} for i in self.items]

    def path(self, lib_id: str) -> Optional[Path]:
        p = (self.root / lib_id).resolve()
        try:
            p.relative_to(self.root.resolve())
        except ValueError:
            return None
        return p if p.is_file() and p.suffix.lower() == ".dbc" else None

    def match(self, seen: Iterable[Key], exclude: Set[str] = frozenset()) -> List[Dict[str, Any]]:
        """Library DBCs ranked by how many of the seen identifiers they define."""
        self.ensure()
        seen = set(seen)
        if not seen:
            return []
        ranked = []
        for i in self.items:
            if i["id"] in exclude or not i["ids"]:
                continue
            hit = seen & i["ids"]
            if not hit:
                continue
            ranked.append({
                "id": i["id"], "product": i["product"], "vendor": i["vendor"], "file": i["file"],
                "matched": len(hit), "messages": len(i["ids"]),
                "coverage": round(len(hit) / len(i["ids"]), 3),     # share of the DBC seen on the bus
                "explains": round(len(hit) / len(seen), 3),         # share of the bus explained
                "ids": sorted(f"0x{k:X}{' EXT' if e else ''}" for k, e in hit)[:12],
            })
        ranked.sort(key=lambda r: (-r["matched"], -r["coverage"], r["messages"]))
        return ranked[:10]
