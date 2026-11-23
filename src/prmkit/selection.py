"""Deterministic representatives of the candidate set; no global PF claims."""
from collections import defaultdict
import itertools

import numpy as np

from .artifacts import candidate_catalog
from .config import checked_keys, number
from .io import atomic_json
from .verification import PASS, attach_verification, current_verification


def nondominated(rows):
    if not rows:
        return []
    costs = np.asarray([r["costs"] for r in rows])
    return [r["path_id"] for i,r in enumerate(rows) if not np.any(np.all(costs <= costs[i],axis=1) & np.any(costs < costs[i],axis=1))]


def representative(catalog, *, only_verified=True, include_extremes=True, include_pairwise_knees=True,
                   include_balanced=True, max_per_query=7):
    number(max_per_query,"selection.max_per_query",1,integer=True)
    for value in (only_verified, include_extremes, include_pairwise_knees, include_balanced):
        if not isinstance(value,bool):
            raise ValueError("Selection switches must be boolean")
    groups=defaultdict(list)
    for row in catalog["paths"]:
        if only_verified and row["verification"]["status"] not in PASS:
            continue
        groups[(row["run"],row["scene_id"],row["query_id"])].append(row)
    selected, reasons = [], {}
    for group in groups.values():
        front=set(nondominated(group))
        rows=sorted([r for r in group if r["path_id"] in front], key=lambda r:r["path_id"])
        costs=np.array([r["costs"] for r in rows]); span=np.ptp(costs,axis=0)
        normalized=(costs-costs.min(axis=0))/np.where(span>0,span,1)
        chosen=[]
        def pick(index, reason):
            identity=rows[int(index)]["path_id"]
            if identity not in chosen and len(chosen)>=max_per_query:
                return
            if identity not in chosen:
                chosen.append(identity)
            reasons.setdefault(identity,[]).append(reason)
        if include_extremes:
            for i,name in enumerate(rows[0]["metrics"]):
                pick(np.argmin(costs[:,i]),"extreme:"+name)
        if include_pairwise_knees:
            for i,j in itertools.combinations(range(costs.shape[1]),2):
                # A documented balanced pairwise compromise, not a curvature oracle.
                pick(np.argmin(np.linalg.norm(normalized[:,[i,j]],axis=1)),f"pairwise_ideal_distance:{i},{j}")
        if include_balanced:
            pick(np.argmin(np.linalg.norm(normalized,axis=1)),"balanced_normalized_ideal_distance")
        selected.extend(chosen)
    return dict(schema_version=1,catalog_hash=catalog["catalog_hash"],path_ids=selected,reasons=reasons,
                mode="representative",only_verified=only_verified,max_per_query=max_per_query,
                policy="Per search/query; candidate nondominated set; min/max normalization; objective extrema, pairwise ideal-distance knee heuristic, balanced ideal distance; deterministic ID ties")


def select_paths(directory, config=None, *, catalog=None, output=None):
    from pathlib import Path
    config=dict(config or {})
    checked_keys(config,["mode","only_verified","include_extremes","include_pairwise_knees","include_balanced","max_per_query"],"selection")
    if config.pop("mode","representative") != "representative":
        raise ValueError("Use Explorer selected_paths.json for manual selections")
    catalog=catalog or candidate_catalog(directory)
    catalog=attach_verification(catalog,current_verification(directory,catalog))
    result=representative(catalog,**config)
    output=Path(output) if output is not None else Path(directory)/"analysis/selected_paths.json"
    atomic_json(output,result)
    return dict(result,file=str(output))
