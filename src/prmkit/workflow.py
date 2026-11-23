"""Optional post-search stages around unchanged candidate outputs."""
from pathlib import Path

from .artifacts import candidate_catalog
from .config import checked_keys
from .explorer import explore
from .selection import select_paths
from .verification import verify_paths, options


def validate_stages(verification=None, selection=None, render=None, export=None):
    """Reject stage-option mistakes before a costly planner run."""
    from .config import number
    if verification is not None:
        config=options(verification)
        if config["enabled"] and config["scope"]=="selected" and not (isinstance(selection,dict) and selection.get("file")):
            raise ValueError("Selected verification needs selection={file: selected_paths.json}")
    if selection is not None:
        if not isinstance(selection,dict):
            raise ValueError("selection must be a mapping")
        if "file" in selection:
            checked_keys(selection,["file"],"selection")
            if not Path(selection["file"]).is_file():
                raise ValueError("Selection file does not exist")
        else:
            from .selection import representative
            config=dict(selection)
            if config.pop("mode","representative")!="representative":
                raise ValueError("Manual selection needs a file from the Explorer")
            checked_keys(config,["only_verified","include_extremes","include_pairwise_knees","include_balanced","max_per_query"],"selection")
            representative({"paths":[],"catalog_hash":"preflight"},**config)
    for key,value in (("render",render),("export",export)):
        if value is None or isinstance(value,bool):
            continue
        allowed=["enabled","output","only_verified"]+(["frames","fps","width","height","mode","backend"] if key=="render" else ["usd"])
        checked_keys(value,allowed,key)
        for flag in ("enabled","only_verified","usd"):
            if flag in value and not isinstance(value[flag],bool):
                raise ValueError(f"{key}.{flag} must be boolean")
        if key=="render":
            if value.get("backend","matplotlib")!="matplotlib":
                raise ValueError("Native Isaac rendering is not implemented; choose matplotlib or export USD for Isaac")
            if value.get("mode","selected") not in ("selected","representative"):
                raise ValueError("render.mode must be selected or representative")
            for field,minimum in (("frames",2),("fps",1),("width",240),("height",180)):
                if field in value:
                    number(value[field],"render."+field,minimum,integer=True)
            if value.get("width",960)%2 or value.get("height",720)%2:
                raise ValueError("MP4 dimensions must be even")


def postprocess(directory, *, verification=None, selection=None, render=None, export=None):
    validate_stages(verification,selection,render,export)
    root=Path(directory)
    catalog=candidate_catalog(root)
    result={"catalog":str(root/"analysis/catalog.json")}
    if verification is not None:
        verification=options(verification)
        selected_file=(selection or {}).get("file") if isinstance(selection,dict) else None
        checked=verify_paths(root,verification,selection=selected_file,catalog=catalog)
        result["verification"]=dict(summary=checked["summary"],job_id=checked["job_id"],complete=checked["complete"])
    result["explorer"]=explore(root,catalog=catalog)
    if selection is not None:
        if isinstance(selection,dict) and "file" in selection:
            checked_keys(selection,["file"],"manual selection")
            from .artifacts import read_selection
            read_selection(selection["file"],catalog)
            result["selection"]={"file":str(selection["file"])}
        else:
            result["selection"]=select_paths(root,selection,catalog=catalog)
    for key,config in (("export",export),("render",render)):
        if config is None or config is False:
            continue
        config={"enabled":config} if isinstance(config,bool) else dict(config)
        if not config.pop("enabled",True):
            continue
        mode=config.pop("mode","selected")
        config.pop("backend",None)
        if mode=="representative":
            result["selection"]=select_paths(root,{"only_verified":config.get("only_verified",True)},catalog=catalog)
        if "selection" not in result:
            result["selection"]=select_paths(root,{"only_verified":True},catalog=catalog)
        from .delivery import export_paths,render_paths
        function=export_paths if key=="export" else render_paths
        result[key]=function(root,result["selection"]["file"],**config)
    result["complete"]=result.get("verification",{}).get("complete",True)
    return result
