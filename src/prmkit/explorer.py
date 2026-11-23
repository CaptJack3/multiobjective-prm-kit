"""Self-contained, linked Pareto plots and scoped manual path selection."""
from collections import defaultdict
import json
from pathlib import Path

from .artifacts import candidate_catalog
from .io import jsonable
from .selection import nondominated, representative
from .verification import PASS, attach_verification, current_verification


def explore(directory, *, catalog=None):
    root=Path(directory).resolve()
    catalog=catalog or candidate_catalog(root)
    verification=current_verification(root,catalog)
    data=attach_verification(catalog,verification)
    groups=defaultdict(list)
    for row in data["paths"]:
        groups[row["run"]].append(row)
    data["verified_candidate_pf"]={key:nondominated([r for r in rows if r["verification"]["status"] in PASS]) for key,rows in groups.items()}
    data["representatives"]=representative(data,only_verified=True)["path_ids"]
    data["candidate_representatives"]=representative(data,only_verified=False)["path_ids"]
    data["verification_summary"]=verification["summary"] if verification else {"unverified":len(data["paths"])}
    payload=json.dumps(jsonable(data),ensure_ascii=False,allow_nan=False).replace("<","\\u003c")
    html=HTML.replace("__DATA__",payload)
    output=root/"explorer/explore.html";output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(html,encoding="utf-8")
    return dict(file=str(output),candidates=len(data["paths"]),verification=data["verification_summary"])


HTML='''<!doctype html><html lang="he" dir="rtl"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Pareto Research Explorer</title>
<style>*{box-sizing:border-box}body{margin:0;background:#f2f6f5;color:#193c40;font:16px/1.6 Arial,sans-serif}main{max-width:1300px;margin:auto;padding:26px 18px}h1{margin:0 0 8px;font-size:30px}p{margin:8px 0}section{background:white;padding:18px;border:1px solid #d4e2de;border-radius:12px;margin:16px 0;min-width:0}select,input,button{font:inherit;max-width:100%;padding:7px;border:1px solid #abc4bd;border-radius:6px}select{width:100%}button{background:#08685e;color:white;cursor:pointer}button.secondary{background:white;color:#18564e}label{display:block;font-size:14px}.filters{display:flex;gap:12px;flex-wrap:wrap;align-items:end}.filters label{flex:1 1 150px}.filters input[type=number]{width:100%}.check{display:flex;align-items:center;gap:7px}.plots{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}.plot{min-width:0}canvas{display:block;width:100%;height:280px;border:1px solid #dae4e0;border-radius:10px;background:white;cursor:crosshair}code{font:12px monospace;direction:ltr;unicode-bidi:isolate;overflow-wrap:anywhere}.scroll{overflow:auto;max-height:500px}table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:9px;border-bottom:1px solid #e1e9e6;text-align:right;vertical-align:top}th{position:sticky;top:0;background:#edf3f0}tr.chosen{background:#fff3cd}tr:hover{background:#edf7f2}.muted{font-size:13px;color:#506966}.notice{background:#fff4d8;border-right:4px solid #ac7c17;padding:12px 16px}.toolbar{display:flex;gap:8px;flex-wrap:wrap}.bad{color:#ad3423}.good{color:#117754}#detail{white-space:pre-wrap;direction:ltr;text-align:left;max-height:250px;overflow:auto;background:#f2f6f5;padding:12px;font-size:12px}button:focus-visible,input:focus-visible,select:focus-visible{outline:3px solid #bb9b36;outline-offset:2px}@media(max-width:950px){.plots{grid-template-columns:1fr}canvas{height:310px}}</style></head><body><main>
<h1>Pareto Research Explorer</h1><p>בחירת מסלולים מתוך תוצאות שמורות — ללא חיפוש נוסף או חיבור לאינטרנט.</p>
<p class="notice">Candidate PF הוא אוסף המועמדים שנמצאו. Verified candidate PF מחושב רק ממועמדים שעברו את הבדיקה המוגדרת; הוא אינו חזית גלובלית מוכחת. <code>discrete_verified</code> מציין דגימות בלבד, ללא הוכחת בטיחות רציפה או פיזית.</p>
<section><p id="summary"></p><label>סביבה · שאילתה · הגדרות חיפוש<select id="group"></select></label><div class="filters">
<label class="check"><input id="verified" type="checkbox">רק מסלולים שעברו אימות</label><label class="check"><input id="invalid" type="checkbox">להציג גם פסילות ושגיאות</label>
<label>עלות מרבית במטרה הראשונה<input id="max-cost" type="number" min="0" step="any" placeholder="ללא הגבלה"></label>
<label>Clearance נצפה מינימלי [מ׳]<input id="min-clearance" type="number" min="0" step="any" placeholder="ללא הגבלה"></label></div>
<p id="counts" class="muted" aria-live="polite"></p><p class="muted">ירוק: עבר בדיקה · אפור: לא נבדק · אדום: נכשל/שגיאה · טבעת צהובה: נבחר. לחצו על נקודה או על תיבת הבחירה בטבלה.</p></section>
<section><div id="plots" class="plots"></div><p id="empty" class="muted"></p></section>
<section><div class="toolbar"><button id="representative" type="button">הוספת מייצגים לקבוצה</button><button id="clear" type="button" class="secondary">ניקוי בחירה</button><button id="download" type="button">הורדת selected_paths.json</button></div><p id="selected-count" aria-live="polite"></p>
<p class="muted">הבחירה נשמרת בין קבוצות. ״מייצגים״ בוחר קצוות, פתרון מאוזן ופשרות זוגיות לפי מרחק מנקודת אידאל מנורמלת; knee הוא כלל היוריסטי. העלו את קובץ הבחירה לשרת והעבירו אותו ל־<code>prmkit render RUN --selection selected_paths.json</code> או ל־<code>prmkit export RUN --selection selected_paths.json</code>.</p>
<div class="scroll"><table><thead><tr><th>בחירה</th><th>Path ID</th><th>עלויות</th><th>אימות</th><th>Clearance [מ׳]</th><th>נקודות</th></tr></thead><tbody id="rows"></tbody></table></div><p id="page-info" class="muted"></p><button id="more" class="secondary" hidden>הצגת 200 נוספים</button>
<details><summary>פרטי המסלול האחרון שנבחר</summary><pre id="detail"></pre></details></section>
<p class="muted">הסרטון הוא שלב נפרד. כל עלויות המטרות ממוזערות; מרחק מפרקים משוקלל הוא מידת אינטגרציה ואינו מטרה רביעית אוטומטית. תוצאות החיפוש המקוריות נשארות ללא שינוי.</p>
</main><script id="data" type="application/json">__DATA__</script><script>
'use strict';
const D=JSON.parse(document.getElementById('data').textContent),$=id=>document.getElementById(id),selected=new Set(),pass=r=>r.verification.status==='discrete_verified'||r.verification.status==='continuous_verified';
let visible=[],limit=200;const maps=new Map();
const groups=[...new Set(D.paths.map(r=>r.run))];
for(const name of groups){const r=D.paths.find(r=>r.run===name),o=document.createElement('option');o.value=name;o.textContent=`${r.scene_id} · ${r.query_id} · N=${r.n} · K=${r.k} · ε=${r.epsilon} · seed=${r.seed}`;$('group').append(o);}
$('verified').checked=D.paths.some(pass);
$('summary').textContent=`${D.paths.length} מועמדים · ${D.paths.filter(pass).length} עברו אימות · ${D.paths.filter(r=>r.verification.status==='unverified').length} לא נבדקו · ${D.experiment_id}`;
function statusColor(r){return pass(r)?'#168475':r.verification.status==='unverified'?'#899b9c':'#bf4234';}
function number(x){return x===null||x===undefined?'—':Number(x).toPrecision(5);}
function choose(row){selected.has(row.path_id)?selected.delete(row.path_id):selected.add(row.path_id);$('detail').textContent=JSON.stringify(row,null,2);render();}
function draw(canvas,x,y){
 const rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1,w=rect.width,h=rect.height;canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);const c=canvas.getContext('2d');c.scale(ratio,ratio);c.clearRect(0,0,w,h);
 const all=D.paths.filter(r=>r.run===$('group').value),metric=all[0]?.metrics||[];if(!all.length)return;
 let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;for(const row of all){xmin=Math.min(xmin,row.costs[x]);xmax=Math.max(xmax,row.costs[x]);ymin=Math.min(ymin,row.costs[y]);ymax=Math.max(ymax,row.costs[y]);}const dx=xmax-xmin||1,dy=ymax-ymin||1;
 c.strokeStyle='#a3b8b1';c.beginPath();c.moveTo(56,18);c.lineTo(56,h-42);c.lineTo(w-16,h-42);c.stroke();c.fillStyle='#314f50';c.font='11px Arial';c.textAlign='center';c.fillText(metric[x]+' ['+(all[0].units[x]||'?')+']',w/2,h-10);c.save();c.translate(12,h/2);c.rotate(-Math.PI/2);c.fillText(metric[y],0,0);c.restore();
 for(let k=0;k<=3;k++){const t=k/3,px=56+t*(w-82),py=h-42-t*(h-68);c.fillText((xmin+t*(xmax-xmin)).toPrecision(3),px,h-26);c.textAlign='right';c.fillText((ymin+t*(ymax-ymin)).toPrecision(3),51,py+3);c.textAlign='center';}
 const points=[];for(const row of visible){const px=56+(row.costs[x]-xmin)/dx*(w-82),py=h-42-(row.costs[y]-ymin)/dy*(h-68);c.beginPath();c.arc(px,py,selected.has(row.path_id)?6:4,0,Math.PI*2);c.fillStyle=statusColor(row);c.fill();if(selected.has(row.path_id)){c.lineWidth=3;c.strokeStyle='#d49d17';c.stroke();}points.push({x:px,y:py,row});}maps.set(canvas,points);
}
function render(){
 const group=D.paths.filter(r=>r.run===$('group').value),verified=$('verified').checked,invalid=$('invalid').checked,mc=$('max-cost').value,clearance=$('min-clearance').value;
 visible=group.filter(r=>(!verified||pass(r))&&(invalid||pass(r)||r.verification.status==='unverified')&&(mc===''||r.costs[0]<=Number(mc))&&(clearance===''||(r.verification.min_clearance_m_observed!=null&&r.verification.min_clearance_m_observed>=Number(clearance))));
 $('counts').textContent=`${visible.length} מתוך ${group.length} מועמדים מוצגים · ${group.filter(pass).length} עברו בדיקה · ${(D.verified_candidate_pf[$('group').value]||[]).length} בחזית המועמדים המאומתים`;
 $('selected-count').textContent=`${selected.size} מסלולים נבחרו בכל הקבוצות`;$('empty').textContent=visible.length?'':'אין מועמדים שעומדים במסננים. אם טרם בוצע אימות, בטלו את ״רק מסלולים שעברו אימות״.';
 $('plots').replaceChildren();maps.clear();let dimensions=group[0]?.metrics.length||0;
 if(dimensions===1){const p=document.createElement('p');p.textContent='מטרה אחת: העלויות זמינות בטבלה.';$('plots').append(p);}
 for(let x=0;x<Math.min(dimensions,3);x++)for(let y=x+1;y<Math.min(dimensions,3);y++){const div=document.createElement('div');div.className='plot';const canvas=document.createElement('canvas');canvas.setAttribute('aria-label',group[0].metrics[x]+' / '+group[0].metrics[y]);div.append(canvas);$('plots').append(div);canvas.addEventListener('click',e=>{const box=canvas.getBoundingClientRect();let best=null,d=144;for(const p of maps.get(canvas)||[]){const dist=(p.x-e.clientX+box.left)**2+(p.y-e.clientY+box.top)**2;if(dist<d){best=p.row;d=dist;}}if(best)choose(best);});draw(canvas,x,y);}
 $('rows').replaceChildren();for(const r of visible.slice(0,limit)){const tr=document.createElement('tr');if(selected.has(r.path_id))tr.className='chosen';const cell=document.createElement('td'),check=document.createElement('input');check.type='checkbox';check.checked=selected.has(r.path_id);check.setAttribute('aria-label',r.path_id);check.addEventListener('change',()=>choose(r));cell.append(check);tr.append(cell);
 for(const value of [r.path_id,r.costs.map((x,i)=>r.metrics[i]+': '+number(x)).join(' · '),r.verification.status,number(r.verification.min_clearance_m_observed),r.waypoints]){const td=document.createElement('td');td.textContent=value;tr.append(td);}$('rows').append(tr);}
 $('more').hidden=limit>=visible.length;$('page-info').textContent=`${Math.min(limit,visible.length)} מתוך ${visible.length} שורות`;
}
for(const id of ['group','verified','invalid','max-cost','min-clearance'])$(id).addEventListener('change',()=>{limit=200;render();});
$('more').addEventListener('click',()=>{limit+=200;render();});
$('clear').addEventListener('click',()=>{selected.clear();render();});
$('representative').addEventListener('click',()=>{const ids=$('verified').checked?D.representatives:D.candidate_representatives;for(const row of visible)if(ids.includes(row.path_id))selected.add(row.path_id);render();});
$('download').addEventListener('click',()=>{const result={schema_version:1,catalog_hash:D.catalog_hash,experiment_id:D.experiment_id,path_ids:[...selected].sort(),mode:'manual',created_utc:new Date().toISOString()};const blob=new Blob([JSON.stringify(result,null,2)+'\\n'],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='selected_paths.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
let timer;window.addEventListener('resize',()=>{clearTimeout(timer);timer=setTimeout(render,120);});render();
</script></body></html>'''
