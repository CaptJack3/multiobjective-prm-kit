"""Portable acceptance checks for the included research examples.
The original research repository is not needed. Requires numpy and usd-core.
"""
from pathlib import Path
from html.parser import HTMLParser
from urllib.parse import urlsplit,unquote
import argparse,hashlib,json,subprocess,sys,shutil,tempfile,importlib.metadata
import numpy as np
from pxr import Usd,UsdUtils,UsdGeom
HERE=Path(__file__).resolve().parent
parser_args=argparse.ArgumentParser(description='Verify example hashes, USD dependencies, route data, geometry reconstruction and portability without PRM or Isaac.')
parser_args.add_argument('--report',type=Path,required=True)
args=parser_args.parse_args()
if args.report.exists():raise FileExistsError(args.report)
report={'status':'PASS','scope':'Asset integrity, archived USD dependencies, numerical geometry reconstruction, portable commands and static HTML links; no new PRM, curve certification, GUI or RTX rendering.'}
source=json.loads((HERE/'SOURCE_MANIFEST.json').read_text())
for item in source:
 assert hashlib.sha256((HERE/item['destination']).read_bytes()).hexdigest()==item['sha256'],item['destination']
report['source_manifest_hashes_verified']=len(source)
catalog=json.loads((HERE/'catalog.json').read_text())
report['archived_usd']=[]
for path in sorted(HERE.rglob('*.usdc')):
 layers,assets,missing=UsdUtils.ComputeAllDependencies(str(path))
 assert not assets and not missing,(path,assets,missing)
 assert len(layers)==1,(path,[l.identifier for l in layers])
 stage=Usd.Stage.Open(str(path))
 assert UsdGeom.GetStageUpAxis(stage)=='Z'
 assert UsdGeom.GetStageMetersPerUnit(stage)==1
 assert stage.GetPrimAtPath('/World/Robot')
 report['archived_usd'].append({'path':str(path.relative_to(HERE)),'prims':sum(1 for _ in stage.Traverse()),'external_dependencies':0})
for row in catalog:
 path=HERE/row['routes_directory']
 metadata=json.loads((path/'manifest.json').read_text())
 query=json.loads((path/'query.json').read_text())
 for n in range(1,4):
  route=f'route_{n:02d}'
  with np.load(path/(route+'.npz'),allow_pickle=False) as z:
   assert z['q'].ndim==2 and z['q'].shape[1]==6
   assert z['ee'].shape==(len(z['q']),3)
   assert np.isfinite(z['q']).all() and np.isfinite(z['ee']).all()
   assert np.allclose(z['q'][0],query['start'],atol=1e-8)
   assert np.allclose(z['q'][-1],query['goal'],atol=1e-8)
   selected=next(s for s in metadata['selected'] if s['id']==route)
   assert np.allclose(z['costs'],selected['costs'],rtol=1e-12,atol=1e-12)
report['saved_routes_checked']=21
class Links(HTMLParser):
 def __init__(self):super().__init__();self.links=[]
 def handle_starttag(self,tag,attrs):
  for key,value in attrs:
   if key in ['href','src','poster'] and value:self.links.append(value)
parser=Links();parser.feed((HERE/'index.html').read_text())
for link in parser.links:
 parsed=urlsplit(link)
 if parsed.scheme or not parsed.path or parsed.path=='PORTABILITY_CHECK.json':continue
 assert (HERE/unquote(parsed.path)).is_file(),link
report['local_gallery_links_verified']=len(parser.links)
# Only the portable package is copied; the old research repository is unavailable
# through any relative path from this temporary root.
with tempfile.TemporaryDirectory(prefix='prmkit_showcase_portability_') as tmp:
 temp=Path(tmp);clean=temp/'example'
 shutil.copytree(HERE,clean)
 command=[sys.executable,str(clean/'generate_household_specs.py'),'--output',str(temp/'specs')]
 subprocess.run(command,cwd=temp,check=True,stdout=subprocess.DEVNULL)
 report['household_geometry_matches_original']={}
 for name in ['kitchen','office','shelves']:
  regenerated=json.loads((temp/'specs'/name/'scene_spec.json').read_text())
  original=json.loads((clean/'scenes'/name/'scene_spec.json').read_text())
  assert regenerated==original,name
  report['household_geometry_matches_original'][name]={'visual_objects':len(original['geometry']),'collision_proxies':len(original['colliders']),'targets':len(original['targets'])}
 subprocess.run([sys.executable,str(clean/'build_scenes.py'),'--output',str(temp/'usd')],cwd=temp,check=True,stdout=subprocess.DEVNULL)
 subprocess.run([sys.executable,str(clean/'build_scenes.py'),'--scene','kitchen','--spec-root',str(temp/'specs'),'--output',str(temp/'regenerated')],cwd=temp,check=True,stdout=subprocess.DEVNULL)
 report['rebuilt_scenes_verified']=[]
 for row in catalog:
  path=temp/'usd'/(row['id']+'.usdc');stage=Usd.Stage.Open(str(path))
  layers,assets,missing=UsdUtils.ComputeAllDependencies(str(path));assert len(layers)==1 and not assets and not missing
  original=Usd.Stage.Open(str(clean/row['static_scene']))
  # A reconstruction must preserve the archived robot exactly.
  for prim in Usd.PrimRange(original.GetPrimAtPath('/World/Robot')):
   rebuilt=stage.GetPrimAtPath(prim.GetPath());assert rebuilt
   for attribute in prim.GetAttributes():
    other=rebuilt.GetAttribute(attribute.GetName());assert other
    assert str(attribute.Get())==str(other.Get()),str(attribute.GetPath())
  if row['id'] in ['kitchen','office','shelves']:
   # Geometry authored from JSON is checked against independent original USD.
   for root in ['/World/Scene','/World/CollisionProxies']:
    for prim in Usd.PrimRange(original.GetPrimAtPath(root)):
     rebuilt=stage.GetPrimAtPath(prim.GetPath());assert rebuilt,str(prim.GetPath())
     for name in ['points','faceVertexCounts','faceVertexIndices','size','radius','height','xformOp:translate','xformOp:scale']:
      a=prim.GetAttribute(name);b=rebuilt.GetAttribute(name)
      if a and a.Get() is not None:
       assert b and b.Get() is not None
       assert np.allclose(np.asarray(a.Get()),np.asarray(b.Get()),rtol=1e-7,atol=1e-8),(row['id'],str(prim.GetPath()),name)
  else:
   spec=json.loads((clean/row['geometry_spec']).read_text())
   for obj in spec['objects']:
    prim=stage.GetPrimAtPath('/World/Scene/'+obj['name'].replace('-','m').replace('.','p'))
    assert np.allclose(prim.GetAttribute('xformOp:translate').Get(),obj['position_m'])
    assert np.allclose(prim.GetAttribute('xformOp:scale').Get(),obj['dimensions_m'])
  report['rebuilt_scenes_verified'].append(row['id'])
 # Refuse existing output and leave it unchanged.
 existing=temp/'usd/kitchen.usdc';before=hashlib.sha256(existing.read_bytes()).hexdigest()
 result=subprocess.run([sys.executable,str(clean/'build_scenes.py'),'--output',str(temp/'usd')],cwd=temp,capture_output=True)
 assert result.returncode!=0
 assert hashlib.sha256(existing.read_bytes()).hexdigest()==before
report['existing_output_protected']=True
# Recheck that opening/authoring USD did not mutate delivered templates.
for item in source:assert hashlib.sha256((HERE/item['destination']).read_bytes()).hexdigest()==item['sha256']
report['dependencies']={name:importlib.metadata.version(name) for name in ['numpy','usd-core']}

report['limitations']=['Live Isaac Sim/RTX and browser playback were not rerun in this session.','Copied historical videos and audits preserve source hashes; no new continuous collision audit is claimed.','Historical optimization, large PRM builds and archived video production are not packaged as one-command reproduction.']
args.report.parent.mkdir(parents=True,exist_ok=True)
args.report.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
