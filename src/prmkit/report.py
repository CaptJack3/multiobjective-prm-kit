"""Offline experiment review with configurable cost axes and path selection."""
import html
import json
import numpy as np
from .io import jsonable


def write_report(directory, config, graph, result, backend):
    from scipy.spatial import ConvexHull, QhullError
    obstacles = {name: [] for name in ("XY", "XZ", "YZ")}
    if hasattr(backend, "vis_geoms"):
        world_vertices = [np.asarray(mesh.vertices) @ geom.placement.rotation.T + geom.placement.translation
                          for mesh, geom in zip(backend.meshes[backend.moving_count:], backend.vis_geoms[backend.moving_count:])]
    else:
        world_vertices = []
        for obj in config["environment"].get("objects", []):
            if not obj.get("collider", True):
                continue
            p, d = np.asarray(obj["position_m"]), np.asarray(obj["dimensions_m"])
            if len(p) == 2:
                p, d = np.r_[p, 0], np.r_[d, 0]
            world_vertices.append(np.array([p + d * np.array([x,y,z])/2 for x in (-1,1) for y in (-1,1) for z in (-1,1)]))
    for name, dims in (("XY", [0,1]), ("XZ", [0,2]), ("YZ", [1,2])):
        for vertices in world_vertices:
            projected = vertices[:, dims]
            try:
                projected = projected[ConvexHull(projected).vertices]
            except QhullError:
                projected = np.unique(projected, axis=0)
            obstacles[name].append(projected.tolist())
    with np.load(directory / "paths.npz", allow_pickle=False) as z:
        payload = dict(costs=result["costs"], names=[m["name"] for m in config["metrics"]],
                       paths=[z["ee"][a:b].tolist() for a, b in zip(z["offsets"][:-1], z["offsets"][1:])],
                       obstacles=obstacles, complete=result["complete"],
                       epsilon=result["epsilon"], termination=result["termination"])
    data = json.dumps(jsonable(payload)).replace("<", "\\u003c")
    title = html.escape(config["environment"].get("environment_id", "PRM experiment"))
    template = r'''<!doctype html><meta charset="utf-8"><title>PRM experiment</title>
<style>body{font:16px system-ui;max-width:1100px;margin:36px auto;padding:0 20px;color:#182a39;background:#f3f6f9}canvas{background:white;border:1px solid #cdd6dd;border-radius:10px;width:100%;max-width:520px}select,input{margin:8px;padding:6px}article{display:flex;gap:20px;flex-wrap:wrap}pre{white-space:pre-wrap}a{color:#155d9f}</style>
<h1>__TITLE__</h1><p id="status"></p><p><a href="front.csv">Cost table</a> · <a href="summary.json">Run summary</a> · <a href="paths.npz">Joint paths (NPZ)</a></p>
<label>Cost X <select id="x"></select></label><label>Cost Y <select id="y"></select></label>
<label>Path <select id="path"></select></label><label>Projection <select id="projection"><option>XY</option><option>XZ</option><option>YZ</option></select></label>
<article><canvas id="front" width="520" height="380"></canvas><canvas id="motion" width="520" height="380"></canvas></article>
<p>The motion panel shows the end-effector polyline and convex projected silhouettes of world obstacles. This is a geometric review, not a time-parameterized robot execution.</p><pre id="detail"></pre>
<script>const D=__DATA__;const $=id=>document.getElementById(id);let selected=0;
D.names.forEach((n,i)=>['x','y'].forEach(id=>{let o=document.createElement('option');o.textContent=n;o.value=i;$(id).append(o)}));$('y').value=Math.min(1,D.names.length-1);
D.paths.forEach((_,i)=>{let o=document.createElement('option');o.textContent=i;o.value=i;$('path').append(o)});
$('status').textContent=`${D.complete?'Completed':'PARTIAL'}: ${D.termination}; epsilon ${D.epsilon}; ${D.paths.length} returned paths.`;
function draw(id,points,segments,boxes){let c=$(id),ctx=c.getContext('2d');ctx.clearRect(0,0,c.width,c.height);let all=points.concat(boxes.flat());if(!all.length)return;
let lo=[0,1].map(j=>Math.min(...all.map(p=>p[j]))),hi=[0,1].map(j=>Math.max(...all.map(p=>p[j])));let xy=p=>[35+(p[0]-lo[0])/Math.max(hi[0]-lo[0],1e-9)*450,345-(p[1]-lo[1])/Math.max(hi[1]-lo[1],1e-9)*310];
ctx.fillStyle='#d1d9e0';boxes.forEach(b=>{ctx.beginPath();b.forEach((p,i)=>i?ctx.lineTo(...xy(p)):ctx.moveTo(...xy(p)));ctx.closePath();ctx.fill()});ctx.strokeStyle='#2874a6';ctx.lineWidth=2;
if(segments){ctx.beginPath();points.forEach((p,i)=>{let q=xy(p);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.stroke()}
points.forEach((p,i)=>{let q=xy(p);ctx.fillStyle=!segments&&i===selected?'#e07023':'#2874a6';ctx.beginPath();ctx.arc(...q,!segments&&i===selected?7:3,0,7);ctx.fill()});ctx.fillStyle='#34495e';ctx.fillText(lo.map(x=>x.toPrecision(3)).join(', '),12,370);ctx.fillText(hi.map(x=>x.toPrecision(3)).join(', '),350,20)}
function render(){selected=Number($('path').value);let x=Number($('x').value),y=Number($('y').value);draw('front',D.costs.map(c=>[c[x],c[y]]),false,[]);let dims={'XY':[0,1],'XZ':[0,2],'YZ':[1,2]}[$('projection').value];
let boxes=D.obstacles[$('projection').value];
draw('motion',(D.paths[selected]||[]).map(p=>[p[dims[0]],p[dims[1]]]),true,boxes);$('detail').textContent=D.costs[selected]?D.names.map((n,i)=>n+': '+D.costs[selected][i]).join('\n'): 'No path returned. A disconnected roadmap does not prove continuous-space infeasibility.'}
['x','y','path','projection'].forEach(id=>$(id).onchange=render);render();</script>'''
    (directory / "report.html").write_text(template.replace("__TITLE__", title).replace("__DATA__", data), encoding="utf-8")
