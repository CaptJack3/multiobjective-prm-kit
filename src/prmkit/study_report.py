"""Offline index for case-runner results; individual reports remain unchanged."""
import json
from .io import jsonable


def write_study_report(path, result):
    payload = json.dumps(jsonable(result), ensure_ascii=False, allow_nan=False).replace("<", "\\u003c")
    html = '''<!doctype html><html lang="he" dir="rtl"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>תוצאות ניסוי PRM</title>
<style>*{box-sizing:border-box}body{overflow-wrap:anywhere;font:16px/1.6 system-ui;margin:32px auto;padding:0 20px;max-width:1200px;background:#f3f6f5;color:#173137}h1{margin-bottom:8px}section{padding:20px;background:white;border:1px solid #d4dfdc;border-radius:12px;margin:18px 0}select{font:inherit;max-width:100%;padding:8px}label{display:inline-block;max-width:100%;margin:8px 0}table{width:100%;border-collapse:collapse;font-size:14px}th,td{text-align:right;padding:9px;border-bottom:1px solid #dce5e2}a{color:#00695e}code{direction:ltr;unicode-bidi:isolate;display:inline-block}.scroll{overflow:auto}.muted{color:#526864}.status{font-weight:600}.error{color:#9a2424}#progress{background:#e4f2ed;padding:14px;border-radius:8px}details{margin:12px 0}button{font:inherit;padding:7px 12px}nav{display:flex;gap:18px;flex-wrap:wrap}</style>
<h1>תוצאות ניסוי PRM</h1><div id="progress" role="status"></div>
<nav><a href="study_summary.json">סיכום JSON</a><a href="results.csv">טבלת CSV</a><a href="queries.json">בנק שאילתות</a><a href="plan.json">תוכנית הניסוי</a></nav>
<p class="muted">כל שאילתה היא משימת תכנון עצמאית. אותם קלטים משמשים בכל הסביבות וההגדרות. מספר המסלולים אינו מספר השאילתות.</p>
<section><h2>בחירת קבוצת השוואה</h2><label>N / K / epsilon / seed <select id="group" aria-label="קבוצת השוואה"></select></label><p id="ranking-rule"></p>
<h3>דירוג סביבות בקבוצה</h3><p class="muted">רק סביבות שכל השאילתות בהן הוערכו עד לסיום, ולפחות אחת מהן הניבה מסלול. קודם מספר השאילתות שנפתרו, ואז ממוצע הציון של השאילתות שנפתרו. תוצאות חלקיות וכשלים אינם מדורגים.</p>
<div class="scroll"><table><thead><tr><th>סביבה</th><th>שאילתות שנפתרו</th><th>ציון ממוצע</th><th>שימוש חוזר</th></tr></thead><tbody id="ranking"></tbody></table></div><p id="empty-rank"></p></section>
<section><h2>כל הרצות התכנון בקבוצה</h2><label>שאילתה <select id="query"><option value="all">כולן</option></select></label><label>סטטוס <select id="status"><option value="all">כולם</option></select></label><p id="count"></p>
<div class="scroll"><table><thead><tr><th>הרצה / סביבה</th><th>שאילתה</th><th>סטטוס</th><th>מסלולים</th><th>ציון</th><th>דוח והגדרות</th></tr></thead><tbody id="runs"></tbody></table></div>
<p><button id="more" hidden>הצגת 200 שורות נוספות</button></p></section>
<details><summary>איך מפרשים כישלון או חיפוש חלקי?</summary><p><code>invalid_query</code>: ההתחלה או היעד אינם תקפים בסביבה זו. <code>no_path_in_roadmap</code>: לא נמצא מסלול בגרף המסוים, ללא הוכחה שאין מסלול רציף. <code>partial</code> או <code>failed</code>: יש לקרוא את סיבת העצירה או השגיאה. <code>complete</code> של הניסוי מציין שכל המשימות הסתיימו, ולא שכל המשימות פתירות.</p></details>
<p class="muted">הדוח פועל ללא אינטרנט. קישורים לתוצרי הריצה דורשים את תיקיית התוצאות המלאה. גרף ועלויות משותפים בין אפסילונים בלבד; N/K, סביבה או שאילתה שונים בונים גרף נוסף.</p>
<script id="data" type="application/json">__DATA__</script><script>
const D=JSON.parse(document.getElementById('data').textContent),$=id=>document.getElementById(id);let limit=200;
const option=(value,label)=>{const e=document.createElement('option');e.value=value;e.textContent=label;return e;};
const cell=(row,text)=>{const c=document.createElement('td');c.textContent=text;row.append(c);return c;};
const link=(parent,url,label)=>{const a=document.createElement('a');a.href=url;a.textContent=label;parent.append(a);return a;};
const score=x=>x===null||x===undefined?'—':Number(x).toPrecision(6);
$('progress').textContent=(D.complete?'הניסוי הושלם':'הניסוי טרם הושלם')+' · '+D.evaluated+' מתוך '+D.requested+' הרצות · '+D.plan.environments+' סביבות · '+D.plan.num_queries+' שאילתות · '+D.plan.roadmaps+' גרפים מתוכננים';
$('ranking-rule').textContent='קריטריון: '+D.rank_by+' · כיוון: '+D.rank_direction+' · סריקה מלאה של הרשימה/הגריד: '+(D.plan.grid_exhausted?'כן':'לא / דגימה אקראית');
D.rankings.forEach((g,i)=>$('group').append(option(i,`N=${g.n} · K=${g.k} · ε=${g.epsilon} · seed=${g.seed}`)));
[...new Set(D.rows.map(r=>r.query_id))].forEach(q=>$('query').append(option(q,q)));
[...new Set(D.rows.map(r=>r.status))].forEach(s=>$('status').append(option(s,s)));
function render(){
 const g=D.rankings[Number($('group').value)];$('ranking').replaceChildren();$('runs').replaceChildren();
 if(!g){$('empty-rank').textContent='טרם הסתיימו הרצות.';$('count').textContent='0 הרצות';return;}
 for(const env of g.environments){const tr=document.createElement('tr');cell(tr,env.environment_id);cell(tr,env.solved_queries+' מתוך '+env.total_queries);cell(tr,score(env.score));link(cell(tr,''),env.environment,'scene.json');$('ranking').append(tr);}
 $('empty-rank').textContent=g.environments.length?'':'אין סביבה זכאית לדירוג בקבוצה זו.';
 const rows=D.rows.filter(r=>r.n===g.n&&r.k===g.k&&r.epsilon===g.epsilon&&r.seed===g.seed&&($('query').value==='all'||r.query_id===$('query').value)&&($('status').value==='all'||r.status===$('status').value));
 for(const row of rows.slice(0,limit)){const tr=document.createElement('tr');cell(tr,row.run_id+' / '+row.environment_id);cell(tr,row.query_id);const status=cell(tr,row.status);if(row.error){const d=document.createElement('details'),s=document.createElement('summary'),p=document.createElement('p');s.textContent='פרטי שגיאה';p.textContent=row.error;d.append(s,p);status.append(d);}cell(tr,row.paths);cell(tr,score(row.score));const c=cell(tr,'');if(row.report){link(c,row.report,'דוח');c.append(' · ');}link(c,row.config,'הגדרות');$('runs').append(tr);}
 $('count').textContent=Math.min(rows.length,limit)+' מתוך '+rows.length+' הרצות מוצגות';$('more').hidden=rows.length<=limit;
}
for(const id of ['group','query','status'])$(id).addEventListener('change',()=>{limit=200;render();});$('more').addEventListener('click',()=>{limit+=200;render();});render();
</script></html>'''
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(html.replace("__DATA__", payload), encoding="utf-8")
    temporary.replace(path)
