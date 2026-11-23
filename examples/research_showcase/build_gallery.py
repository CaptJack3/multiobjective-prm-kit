"""Build the offline research-example gallery from local metadata."""
import csv
import html
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main():
    catalog = json.loads((HERE / 'catalog.json').read_text())
    with (HERE / 'metrics.csv').open() as stream:
        metrics = list(csv.DictReader(stream))
    sections = []
    for row in catalog:
        name = row['id']
        directory = row['routes_directory']
        animated = (f' · <a href="{row["motion_scene"]}">Animated USD (original rig)</a>'
                    if row['motion_scene'] else '')
        spec = row['geometry_spec']
        routes = []
        for item in [m for m in metrics if m['scene'] == name]:
            route = item['route']
            label = {'length': 'Shorter path', 'exposure': 'Less obstacle exposure',
                     'visibility': 'Better wrist visibility'}[item['preference']]
            routes.append(f'''<article><h3>{label}</h3>
              <video controls preload="none" poster="{directory}/preview.jpg" src="{directory}/{route}.mp4"></video>
              <p>Length: {float(item['tcp_length_m']):.3f} m · Exposure: {float(item['exposure']):.3f} · Visibility loss: {float(item['visibility_loss']):.3f}</p>
              <a href="{directory}/{route}.npz">Route data (NPZ)</a></article>''')
        sections.append(f'''<section id="{name}"><h2>{html.escape(row['title'])}</h2>
          <div class="scene"><a href="{row['preview']}"><img loading="lazy" src="{row['preview']}" alt="Original scene preview: {name}"></a>
          <div><p><a href="{row['static_scene']}">Open original USD in Isaac Sim</a>{animated}</p>
          <p><a href="{spec}">Editable geometry JSON</a> · <a href="scenes/{name}/scene.json">Planning geometry</a></p>
          <p>Original scene: {row['static_date']}, UR5e with earlier camera/bracket, bare tool flange.</p>
          <p>The three videos below: 21 November 2026, updated gripper + PickNik + D435 assembly.</p>
          <p>{html.escape(row['routes_scope'])}</p>
          <p><a href="{directory}/manifest.json">Recorded results</a> · <a href="{directory}/query.json">Start / goal</a> · <a href="{directory}/dense_audit.json">Archived audit</a></p></div></div>
          <div class="routes">{''.join(routes)}</div></section>''')
    text = '''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>PRM Kit — research examples</title>
<style>body{font:16px/1.55 system-ui,sans-serif;margin:0;background:#f3f5f7;color:#223246}main{max-width:1280px;margin:auto;padding:30px}h1,h2,h3{line-height:1.2}a{color:#16639c}header,section{background:white;padding:24px;margin-bottom:24px;border-radius:12px;border:1px solid #dbe2e9}nav{display:flex;flex-wrap:wrap;gap:16px}img,video{max-width:100%;height:auto;border-radius:6px}.scene{display:grid;grid-template-columns:1fr 1fr;gap:24px}.routes{display:grid;grid-template-columns:repeat(3,1fr);gap:18px}.routes p{font-size:14px}.note{padding:12px;background:#eff5fa}code,pre{background:#f0f3f6;padding:8px;white-space:pre-wrap;overflow-wrap:anywhere}@media(max-width:800px){.routes,.scene{grid-template-columns:1fr}main{padding:12px}}</style>
<main><header><p>RESEARCH EXAMPLES · PORTABLE LAB HANDOFF</p><h1>Scenes, saved paths and comparisons</h1>
<p dir="rtl" lang="he">מטבח, עמדת עבודה ומדפים — לצד E4/E5/E7 ושישה קירות. קובצי סביבה, קוד יצירה, מסלולים שמורים וסרטונים מהעבודה הקודמת.</p>
<p>Seven editable environments · 21 saved preference videos · one before/after smoothing example.</p>
<p><a href="README.md">Instructions and commands</a> · <a href="metrics.csv">All route costs (CSV)</a> · <a href="../../docs/PRMKIT_CAPABILITIES_HE.pdf">Core toolkit: one-page Hebrew PDF</a></p>
<p class="note">No installation is needed to view this gallery. Keep its folder together. The USD scenes use the original hardware; the later preference videos use the updated assembly. These are historical kinematic demonstrations, with each experiment's scope stated below.</p>
<nav>''' + ''.join(f'<a href="#{r["id"]}">{html.escape(r["title"])}</a>' for r in catalog) + '<a href="#smoothing">Smoothing</a></nav></header>' + ''.join(sections) + '''
<section id="smoothing"><h2>Before and after smoothing — E4 shortest path</h2>
<p>18 October 2026 · earlier hardware. A selected saved graph path and its smoothed alternative, with fixed endpoints. Presentation timing is not execution speed.</p>
<video controls preload="none" poster="smoothing/E4_shortest/SNAPSHOTS.jpg" src="smoothing/E4_shortest/BEFORE_AFTER.mp4"></video>
<div class="scene"><img loading="lazy" src="smoothing/E4_shortest/EE_BEFORE_AFTER.png" alt="Before and after TCP paths"><img loading="lazy" src="smoothing/E4_shortest/SAFETY_VISIBILITY.png" alt="Archived clearance and visibility comparison"></div>
<p><a href="smoothing/E4_shortest/data/original_replay.npz">Original replay</a> · <a href="smoothing/E4_shortest/data/smoothed_replay.npz">Smoothed replay</a> · <a href="smoothing/E4_shortest/selection.json">Selection metadata</a></p></section>
<footer>Source hashes: <a href="SOURCE_MANIFEST.json">manifest</a> · Delivery checks: <a href="PORTABILITY_CHECK.json">portability report</a></footer></main></html>'''
    (HERE / 'index.html').write_text(text, encoding='utf-8')
    print(HERE / 'index.html')


if __name__ == '__main__':
    main()
