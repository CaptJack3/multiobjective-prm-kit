"""Portable files, explicit plug-ins, and deterministic input fingerprints."""
from __future__ import annotations

import hashlib
import importlib
import importlib.util
import json
import os
import sys
from pathlib import Path

import numpy as np


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def jsonable(obj):
    if isinstance(obj, np.ndarray):
        return jsonable(obj.tolist())
    if isinstance(obj, np.generic):
        return jsonable(obj.item())
    if isinstance(obj, float) and not np.isfinite(obj):
        return None
    if isinstance(obj, Path):
        return str(obj)
    if isinstance(obj, dict):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (tuple, list)):
        return [jsonable(v) for v in obj]
    return obj


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(jsonable(value), f, indent=2, allow_nan=False, sort_keys=True)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    tmp.replace(path)


def atomic_npz(path, **arrays):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("wb") as f:
        np.savez_compressed(f, **arrays)
        f.flush()
        os.fsync(f.fileno())
    tmp.replace(path)


def plugin(spec, base):
    """Explicit local file.py:function or installed.module:function (trusted code)."""
    module, sep, name = spec.rpartition(":")
    if not sep or not module or not name:
        raise ValueError("Plug-in must be file.py:function or installed.module:function")
    if module.endswith(".py"):
        path = (Path(base) / module).resolve()
        entry = importlib.util.spec_from_file_location("prmkit_user_" + digest(path), path)
        loaded = importlib.util.module_from_spec(entry)
        sys.modules[entry.name] = loaded
        entry.loader.exec_module(loaded)
    else:
        loaded = importlib.import_module(module)
    return getattr(loaded, name)

