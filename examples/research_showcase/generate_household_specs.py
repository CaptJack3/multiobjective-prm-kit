"""Portable adaptation of experiments/demo_extensions/dense/build_specs.py.
Only output handling and planning-JSON projection changed; procedural geometry is unchanged.
Run with Python 3.10+, with no third-party dependencies.
"""
"""Small, editable, dimensioned household scenes; no external asset downloads."""
from pathlib import Path
import argparse
import json, math, copy

OUT = None  # Selected by --output; never writes into the bundled examples.
MATERIALS={
 'ivory':([.76,.75,.68],.06,.42), 'white':([.88,.90,.87],0,.32),
 'wood':([.28,.13,.055],0,.48), 'oak':([.55,.34,.15],0,.55),
 'teal':([.045,.25,.23],.15,.40), 'navy':([.035,.085,.14],.05,.4),
 'black':([.018,.025,.03],.08,.38), 'metal':([.47,.50,.53],.82,.23),
 'glass':([.10,.23,.24],.35,.12), 'terracotta':([.52,.19,.095],0,.60),
 'green':([.075,.24,.065],0,.6), 'leaf':([.17,.36,.09],0,.55),
 'yellow':([.88,.59,.035],0,.36), 'red':([.80,.075,.055],0,.32),
 'blue':([.04,.38,.85],.05,.32), 'purple':([.48,.12,.70],0,.32),
 'paper':([.85,.82,.70],0,.75), 'screen':([.035,.14,.19],.15,.2),
 'floor':([.35,.37,.36],0,.7), 'grout':([.16,.18,.18],0,.8),
}

class Builder:
 def __init__(self,name,title):
  self.s={'name':name,'title':title,'units':'meters','robot_base_world_m':[0,-.08,.86],
   'materials':MATERIALS,'geometry':[],'colliders':[],'targets':[],
   'motion_semantics':'Kinematic reach demonstration; no grasp or dynamic control.',
   'provenance':'Original procedural household scene; native UR5e visual CAD reused locally.'}
 def obj(self,name,shape,pos,size,mat,**kw):
  self.s['geometry'].append(dict(name=name,shape=shape,position_m=list(pos),size=list(size),material=mat,**kw))
 def box(self,n,p,s,m):self.obj(n,'box',p,s,m)
 def cyl(self,n,p,r,h,m):self.obj(n,'cylinder',p,[r,h],m)
 def sphere(self,n,p,s,m):self.obj(n,'sphere',p,s,m)
 def proxy(self,n,p,s,category='obstacle'):
  self.s['colliders'].append(dict(name=n,position_m=list(p),dimensions_m=list(s),category=category,collider=True))
 def lathe(self,n,p,profile,m):self.obj(n,'lathe',p,[1,1,1],m,profile=profile)
 def tube(self,n,points,r,m):self.obj(n,'tube',[0,0,0],[r],m,points=points)
 def room(self):
  self.box('floor',(0,0,-.045),(5,5,.08),'floor')
  for x in range(-5,6):
   self.box(f'floor_line_x{x}',(x*.4,0,-.003),(.003,4,.001),'grout')
   self.box(f'floor_line_y{x}',(0,x*.4,-.003),(4,.003,.001),'grout')
  self.box('back_wall',(.35,.94,1.25),(3.0,.08,2.5),'ivory')
  self.box('wall_baseboard',(.35,.887,.08),(3,.025,.15),'white')
  self.proxy('floor',(0,0,-.045),(5,5,.08))
  self.proxy('back_wall_and_tiles',(.35,.928,1.25),(3,.110,2.5))
 def desk(self,kitchen=False):
  self.box('worktop',(.33,0,.815),(1.52,1.20,.07),'ivory' if kitchen else 'oak')
  self.proxy('worktop',(.33,0,.815),(1.52,1.20,.07),'table')
  # Base is mounted 10 mm above the worktop on a bolted plate.
  self.cyl('robot_mount',(0,-.08,.855),.09,.01,'metal')
  for x in [-.06,.06]:
   for y in [-.14,-.02]:self.cyl(f'bolt_{x}_{y}',(x,y,.864),.005,.008,'black')
  if kitchen:
   self.box('cabinet_body',(.33,.02,.42),(1.48,1.12,.74),'teal')
   for i,x in enumerate([-.20,.17,.54,.91]):
    self.box(f'cabinet_door{i}',(x,-.552,.43),(.354,.028,.68),'teal')
    self.box(f'cabinet_inset{i}',(x,-.57,.43),(.305,.009,.60),'navy')
    self.box(f'cabinet_panel{i}',(x,-.577,.43),(.27,.01,.56),'teal')
    self.box(f'handle{i}',(x,-.598,.685),(.14,.018,.012),'metal')
   self.box('cabinet_toekick',(.33,-.50,.07),(1.48,.08,.10),'black')
   self.proxy('cabinet_and_handles',(.33,0,.40),(1.52,1.22,.80),'table')
  else:
   for i,(x,y) in enumerate([(-.31,-.48),(-.31,.48),(.97,-.48),(.97,.48)]):
    self.box(f'table_leg{i}',(x,y,.39),(.045,.045,.78),'navy')
    self.proxy(f'table_leg{i}',(x,y,.39),(.045,.045,.78),'table')
   self.box('lower_crossbar',(.33,.48,.18),(1.30,.035,.05),'navy')
   self.proxy('lower_crossbar',(.33,.48,.18),(1.30,.035,.05),'table')
 def bottle(self,n,x,y,z,h=.22,r=.031,mat='glass'):
  self.lathe(n,(x,y,z),[(0,0),(r,0),(r,h*.65),(r*.45,h*.79),(r*.45,h*.95),(r*.5,h*.96),(r*.5,h),(0,h)],mat)
  self.cyl(n+'_label',(x,y,z+h*.42),r*1.015,h*.23,'paper')
  self.cyl(n+'_cap',(x,y,z+h*1.01),r*.52,h*.07,'metal')
  self.proxy(n,(x,y,z+h*.54),(r*2.08,r*2.08,h*1.08))
 def mug(self,n,x,y,z,mat='white'):
  self.lathe(n,(x,y,z),[(0,0),(.034,0),(.039,.079),(.034,.083),(.029,.077),(.025,.008),(0,.008)],mat)
  pts=[[x+.043+.026*math.cos(a),y,z+.043+.030*math.sin(a)] for a in [(-math.pi/2+i*math.pi/20) for i in range(41)]]
  self.tube(n+'_handle',pts,.006,mat)
  self.proxy(n,(x+.018,y,z+.043),(.120,.080,.086))
 def pot(self,n,x,y,z):
  self.lathe(n,(x,y,z),[(0,0),(.042,0),(.056,.095),(.054,.10),(0,.10)],'terracotta')
  self.cyl(n+'_soil',(x,y,z+.10),.048,.003,'wood')
  for i in range(9):
   a=i*2.399;r=.035+.017*(i%2)
   self.tube(n+f'_stem{i}',[[x,y,z+.10],[x+r*math.cos(a),y+r*math.sin(a),z+.20+(i%3)*.015]],.002,'green')
   self.sphere(n+f'_leaf{i}',(x+r*math.cos(a),y+r*math.sin(a),z+.19+(i%3)*.015),(.025,.016,.055),'leaf' if i%2 else 'green')
  self.proxy(n,(x,y,z+.15),(.18,.18,.30))
 def book(self,n,x,y,z,w=.14,d=.18,h=.025,mat='navy'):
  self.box(n+'_pages',(x,y,z+h/2),(w*.96,d*.96,h*.78),'paper')
  self.box(n+'_top',(x,y,z+h),(w,d,.003),mat);self.box(n+'_bottom',(x,y,z),(w,d,.003),mat)
  self.box(n+'_spine',(x-w/2,y,z+h/2),(.004,d,h),mat)
  self.proxy(n,(x,y,z+h/2),(w+.006,d+.002,h+.006))
 def target(self,n,p,mat):
  self.box(n,p,(.035,.035,.035),mat)
  self.cyl(n+'_pad',(p[0],p[1],p[2]-.020),.029,.005,'black')
  self.s['targets'].append({'name':n,'marker_world_m':list(p),'tcp_world_m':[p[0],p[1],p[2]+.080],'color':mat,'collider':False})
 def save(self):
  out=OUT/self.s['name'];out.mkdir(parents=True,exist_ok=True)
  (out/'scene_spec.json').write_text(json.dumps(self.s,indent=2)+'\n')
  base=self.s['robot_base_world_m']
  objects=copy.deepcopy(self.s['colliders'])
  for obj in objects:obj['position_m']=[x-b for x,b in zip(obj['position_m'],base)]
  for target in self.s['targets']:
   objects.append(dict(name=target['name'],category='target',collider=False,
    dimensions_m=[.035,.035,.035],position_m=[x-b for x,b in zip(target['marker_world_m'],base)]))
  target=self.s['targets'][0]
  env=dict(environment_id=self.s['name'],description=self.s['title'],objects=objects,
   target=dict(object=target['name'],tcp_target_m=[x-b for x,b in zip(target['tcp_world_m'],base)]),
   robot_base_world_m=base,coordinate_frame='robot base; scene_spec.json uses world coordinates')
  (out/'scene.json').write_text(json.dumps(env,indent=2)+'\n')
  return self.s

def kitchen():
 b=Builder('kitchen','Kitchen / cluttered preparation station');b.room();b.desk(True)
 # Backsplash tiles and a real composed preparation area.
 for i in range(9):
  for j in range(3):b.box(f'backsplash_{i}_{j}',(-.35+i*.17,.884,.93+j*.12),(.163,.014,.114),'white')
 b.box('cutting_board',(.55,-.24,.860),(.32,.23,.020),'wood');b.proxy('cutting_board',(.55,-.24,.860),(.32,.23,.02))
 b.box('knife_handle',(.60,-.30,.883),(.10,.025,.018),'black');b.box('knife_blade',(.72,-.30,.881),(.14,.023,.004),'metal')
 b.proxy('knife',(.66,-.30,.883),(.27,.03,.02))
 for i,(x,y,h) in enumerate([(.82,.32,.25),(.94,.25,.20),(.78,.47,.18),(.95,.46,.27)]):b.bottle(f'bottle{i}',x,y,.85,h,mat='glass' if i%2 else 'teal')
 b.lathe('bowl',(.71,.05,.85),[(0,0),(.05,0),(.105,.075),(.10,.08),(.09,.072),(.043,.012),(0,.012)],'white')
 b.proxy('bowl_and_fruit',(.71,.05,.902),(.21,.21,.104))
 for i,(x,y) in enumerate([(.67,.02),(.73,.03),(.70,.08)]):b.sphere(f'fruit{i}',(x,y,.917),(.034,.035,.033),'red' if i==0 else 'yellow')
 b.mug('coffee',.24,.33,.85,'terracotta');b.mug('tea',.44,.43,.85,'white')
 b.pot('herb',.00,.45,.85)
 for i in range(4):b.cyl(f'plate{i}',(.92,-.23,.856+i*.009),.090,.008,'white')
 b.proxy('plate_stack',(.92,-.23,.876),(.18,.18,.05))
 # Slatted dish rack creates visual occlusion while maintaining reach corridors.
 for i in range(5):b.box(f'rack_slat{i}',(.45+i*.035,.22,.92),(.006,.11,.14),'wood')
 b.proxy('dish_rack',(.52,.22,.945),(.16,.15,.19))
 for i in range(4):b.obj(f'drying_plate{i}','cylinder',(.467+i*.035,.22,.968),[.070,.006],'white',axis='X')
 # Tall worktop objects create actual approach and line-of-sight constraints.
 b.lathe('utensil_crock',(.25,-.33,.85),[(0,0),(.048,0),(.055,.15),(.048,.155),(.042,.145),(.038,.012),(0,.012)],'terracotta')
 for i in range(5):
  xx=.225+(i%3)*.024;yy=-.35+(i//3)*.035
  b.cyl(f'utensil_stem{i}',(xx,yy,1.06),.004,.28,'wood')
  b.sphere(f'utensil_head{i}',(xx,yy,1.21+(i%2)*.025),(.015,.009,.033),'wood')
 b.proxy('utensil_crock_and_tools',(.25,-.33,1.0625),(.12,.13,.425))
 b.lathe('kettle',(.24,.12,.85),[(0,0),(.068,0),(.075,.09),(.055,.16),(.025,.18),(0,.18)],'metal')
 b.cyl('kettle_knob',(.24,.12,1.041),.013,.022,'black')
 b.tube('kettle_handle',[[.24,.12+.09*math.cos(a),.98+.09*math.sin(a)] for a in [i*math.pi/24 for i in range(25)]],.008,'black')
 b.tube('kettle_spout',[[.29,.12,.91],[.34,.12,.97],[.36,.12,1.0]],.014,'metal')
 b.proxy('kettle',(.27,.12,.965),(.22,.20,.230))
 b.box('coffee_grinder',(.65,-.43,.945),(.13,.14,.19),'navy')
 b.cyl('grinder_hopper',(.65,-.43,1.083),.052,.095,'glass');b.cyl('grinder_lid',(.65,-.43,1.134),.057,.012,'black')
 b.box('grinder_label',(.65,-.503,.98),(.075,.004,.035),'metal');b.proxy('coffee_grinder',(.65,-.43,.996),(.14,.15,.292))
 b.bottle('olive_oil',-.23,.28,.85,.29,.039,'glass')
 b.lathe('storage_jar',(-.23,-.35,.85),[(0,0),(.055,0),(.055,.17),(0,.17)],'ivory');b.cyl('storage_jar_lid',(-.23,-.35,1.026),.059,.012,'wood');b.proxy('storage_jar',(-.23,-.35,.942),(.12,.12,.184))
 b.cyl('rear_target_jar',(.58,.37,.9175),.040,.135,'ivory');b.cyl('rear_target_lid',(.58,.37,.9875),.042,.005,'wood');b.proxy('rear_target_jar',(.58,.37,.920),(.086,.086,.14))
 b.cyl('front_target_jar',(.65,-.10,.941),.035,.142,'teal');b.cyl('front_target_lid',(.65,-.10,1.015),.037,.005,'wood');b.proxy('front_target_jar',(.65,-.10,.944),(.078,.078,.148))
 b.target('Target_A',(.47,-.08,.887),'blue');b.target('Target_B',(.58,.37,1.013),'yellow');b.target('Target_C',(.65,-.10,1.040),'purple')
 return b.save()

def office():
 b=Builder('office','Office / dense desktop workspace');b.room();b.desk()
 b.box('monitor_stand',(.62,.36,.861),(.25,.16,.022),'black');b.box('monitor_neck',(.62,.41,1.01),(.045,.045,.29),'metal')
 b.box('monitor_case',(.62,.42,1.23),(.52,.045,.34),'black');b.box('monitor_screen',(.62,.393,1.23),(.485,.004,.303),'screen')
 b.proxy('monitor',(.62,.37,1.125),(.53,.19,.55))
 for i in range(6):b.box(f'screen_line{i}',(.50+.013*i,.389,1.31-i*.033),(.19+.013*(i%3),.001,.006),'teal')
 b.box('keyboard',(.67,.10,.865),(.35,.14,.025),'navy');b.proxy('keyboard',(.67,.10,.868),(.35,.14,.036))
 for row in range(4):
  for col in range(12):b.box(f'key_{row}_{col}',(.515+col*.027,.047+row*.030,.881),(.022,.023,.009),'white' if row==0 else 'black')
 b.sphere('mouse',(.93,.07,.871),(.035,.055,.018),'black');b.proxy('mouse',(.93,.07,.872),(.07,.11,.042))
 for i in range(4):b.book(f'book{i}',.23,.35,.855+i*.028,.18,.22,.026,['navy','terracotta','teal','paper'][i])
 b.mug('mug',.84,-.23,.85,'yellow');b.pot('plant',.96,.47,.85)
 b.cyl('pen_pot',(.42,.43,.90),.035,.10,'metal');b.proxy('pen_pot',(.42,.43,.94),(.09,.09,.18))
 for i in range(7):b.cyl(f'pen{i}',(.40+(i%3)*.015,.415+(i//3)*.015,.985),.003,.13,['blue','red','black'][i%3])
 b.book('notebook',.28,-.40,.855,.22,.16,.018,'terracotta')
 b.box('papertray',(.92,-.42,.89),(.22,.18,.08),'teal');b.box('papers',(.92,-.42,.935),(.205,.16,.014),'paper');b.proxy('papertray',(.92,-.42,.895),(.23,.19,.10))
 b.box('desk_lamp_base',(-.19,.35,.86),(.14,.14,.02),'black');b.tube('lamp_stem',[[-.19,.35,.87],[-.19,.35,1.28],[-.03,.35,1.36]],.012,'metal');b.proxy('lamp_complete',(-.105,.35,1.125),(.31,.15,.55))
 b.lathe('lamp_shade',(-.03,.35,1.31),[(.07,0),(.07,.01),(.035,.09),(0,.09)],'ivory')
 # Upright folders and near-field desk equipment produce genuine occlusion.
 b.box('document_rack_base',(.62,-.40,.862),(.24,.20,.024),'navy')
 for i in range(5):
  x=.53+i*.045;h=.20+(i%2)*.045
  b.box(f'folder{i}',(x,-.40,.877+h/2),(.025,.17,h),['teal','terracotta','navy','ivory','teal'][i])
  b.box(f'folder_spine_label{i}',(x,-.487,.95),(.018,.003,.06),'paper')
 b.proxy('document_rack',(.62,-.40,.998),(.25,.21,.296))
 for i,(x,y) in enumerate([(.36,.27),(.98,.19)]):
  b.box(f'speaker{i}',(x,y,.955),(.075,.09,.21),'black')
  for j,z in enumerate([.91,.99]):b.obj(f'speaker_cone{i}_{j}','cylinder',(x,y-.047,z),[.024,.004],'metal',axis='Y')
  b.proxy(f'speaker{i}',(x,y,.955),(.08,.10,.21))
 b.box('laptop_bottom',(-.20,-.36,.86),(.28,.22,.02),'metal');b.box('laptop_lid',(-.20,-.36,.875),(.28,.22,.01),'navy');b.proxy('closed_laptop',(-.20,-.36,.865),(.29,.23,.035))
 b.bottle('water_bottle',.05,.47,.85,.28,.035,'teal')
 b.box('desktop_frame',(.26,-.18,1.0),(.14,.022,.30),'wood');b.box('frame_paper',(.26,-.193,1.0),(.12,.002,.27),'paper');b.proxy('desktop_frame',(.26,-.18,1.0),(.15,.03,.30))
 b.target('Target_A',(.48,-.11,.873),'blue');b.target('Target_B',(.37,.15,.873),'yellow');b.target('Target_C',(.70,-.12,.873),'purple')
 return b.save()

def shelves():
 b=Builder('shelves','Multilevel workcell / shelves and occlusion');b.room();b.desk()
 b.box('rear_shelf_support',(.63,.62,.815),(.81,.12,.07),'oak');b.proxy('rear_shelf_support',(.63,.62,.815),(.81,.12,.07),'table')
 # Shelves sit at the back of the reachable workspace; the front stays open.
 for i,x in enumerate([.28,.98]):
  for j,y in enumerate([.29,.62]):b.box(f'upright{i}_{j}',(x,y,1.25),(.035,.035,.80),'metal');b.proxy(f'upright{i}_{j}',(x,y,1.25),(.035,.035,.80))
 for k,z in enumerate([.87,1.15,1.45]):
  b.box(f'shelf{k}',(.63,.455,z),(.76,.39,.025),'wood');b.proxy(f'shelf{k}',(.63,.455,z),(.76,.39,.025))
 b.box('back_panel',(.63,.64,1.16),(.76,.018,.62),'navy');b.proxy('back_panel',(.63,.64,1.16),(.76,.018,.62))
 # Individual containers leave front-access openings and obstruct oblique views.
 for i,(x,y,z,h,m) in enumerate([(.38,.49,.885,.18,'teal'),(.83,.51,.885,.20,'ivory'),(.40,.51,1.165,.20,'terracotta'),(.86,.50,1.165,.17,'navy')]):
  b.box(f'bin{i}',(x,y,z+h/2),(.16,.21,h),m);b.box(f'bin_label{i}',(x,y-.107,z+h*.60),(.09,.004,.035),'paper');b.proxy(f'bin{i}',(x,y,z+h/2),(.16,.22,h))
 for i in range(3):b.bottle(f'jar{i}',.40+i*.13,.48,1.465,.12,.038,['teal','ivory','terracotta'][i])
 b.book('manual',.78,-.33,.855,.22,.18,.035,'navy');b.pot('plant',1.01,-.05,.85)
 b.mug('workcup',.16,.38,.85,'white')
 b.box('toolcase',(.58,-.38,.90),(.21,.13,.10),'teal');b.box('toolcase_latch',(.58,-.45,.91),(.05,.015,.03),'metal');b.proxy('toolcase',(.58,-.38,.90),(.22,.16,.10))
 b.box('shelf_divider',(.60,.49,1.30),(.022,.28,.28),'wood');b.proxy('shelf_divider',(.60,.49,1.30),(.022,.28,.28))
 for i in range(2):
  b.box(f'front_storage{i}',(.80,-.12,.925+i*.145),(.20,.16,.14),'ivory' if i==0 else 'terracotta')
  b.box(f'front_storage_label{i}',(.80,-.202,.925+i*.145),(.09,.004,.035),'paper')
 b.proxy('front_storage_stack',(.80,-.12,.998),(.20,.17,.296))
 for i in range(3):b.bottle(f'workcell_canister{i}',-.23+i*.12,.34,.85,.21+(i%2)*.05,.040,['ivory','teal','navy'][i])
 b.box('front_organizer',(.23,-.37,.94),(.20,.15,.18),'navy');b.box('organizer_label',(.23,-.447,.94),(.10,.004,.04),'paper');b.proxy('front_organizer',(.23,-.37,.94),(.21,.16,.18))
 b.target('Target_A',(.55,.28,.909),'blue');b.target('Target_B',(.65,.29,1.190),'yellow');b.target('Target_C',(.42,-.08,.873),'purple')
 return b.save()

if __name__=='__main__':
 parser=argparse.ArgumentParser(description='Regenerate the original three household geometry specifications; no PRM or Isaac required.')
 parser.add_argument('--output',type=Path,required=True)
 OUT=parser.parse_args().output
 OUT.mkdir(parents=True,exist_ok=False)
 ss=[kitchen(),office(),shelves()]
 (OUT/'scene_catalog.json').write_text(json.dumps([{'name':s['name'],'title':s['title'],'visual_parts':len(s['geometry']),'collision_boxes':len(s['colliders']),'targets':len(s['targets'])} for s in ss],indent=2)+'\n')
 print((OUT/'scene_catalog.json').read_text())
