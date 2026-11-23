"""Versioned, conservative complete assembly model for the clearance rebuild.

Robot link convex envelopes contain BOTH historical collision and video meshes.
Attached parts are enclosing primitives. Only rigid same-link assembly contacts
and the existing six adjacent robot pairs / fixed base-table mount are excluded.
"""
from pathlib import Path
import json, sys, time
import numpy as np
import coal
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'results/clearance_rebuild_3env_20260921'
sys.path.insert(0, str(ROOT/'experiments/dense_prm_pipeline'))
from core import EnvironmentCoalChecker, LINKS

def convex(v, f):
    vv=coal.StdVec_Vec3s(); ff=coal.StdVec_Triangle()
    for p in v: vv.append(np.asarray(p, float))
    for t in f: ff.append(coal.Triangle(*map(int,t)))
    return coal.Convex(vv, ff)

def along(a,b):
    a=np.array(a,float); b=np.array(b,float); z=(b-a)/np.linalg.norm(b-a)
    seed=np.eye(3)[np.argmin(abs(z))]; x=np.cross(seed,z); x/=np.linalg.norm(x)
    return pin.SE3(np.column_stack((x,np.cross(z,x),z)), (a+b)/2)

def hardware():
    # Fixed original optical pose, camera axes right/down/forward in wrist frame.
    R=np.array([[0.,0,1],[1,0,0],[0,1,0]]); p=np.array([.1,0,.03])
    # Contains rounded body, fins, USB connector, front panel, lenses and LED.
    lo=np.array([-.043,-.014,-.0285]); hi=np.array([.0455,.014,.00095])
    parts=[('camera_envelope', coal.Box(*(hi-lo)), pin.SE3(R,p+R@((lo+hi)/2)))]
    # Plate and screw heads enclosed together; a rigid part of wrist_3.
    parts.append(('flange_plate_screws',coal.Cylinder(.032,.008),pin.SE3(np.eye(3),np.array([0,0,.004]))))
    a=np.array([-.022,.018,.004]); b=p+R@np.array([0,0,-.025])
    # Reproduce the rendered bracket orientation, not an arbitrary roll.
    z=(b-a)/np.linalg.norm(b-a);x=np.cross([0,0,1],z);x/=np.linalg.norm(x)
    parts.append(('bracket',coal.Box(.022,.006,float(np.linalg.norm(b-a))),pin.SE3(np.column_stack((x,np.cross(z,x),z)),(a+b)/2)))
    a=p+R@np.array([.044,0,-.018])
    # Route on the outward (+wrist Z) side and terminate at the flange adapter.
    # The old cosmetic route ended behind the flange, only 7.2 mm from wrist_2.
    pts=[a,a+np.array([.012,-.018,.015]),np.array([.08,-.025,.035]),np.array([.025,-.020,.025])]
    for i,(a,b) in enumerate(zip(pts[:-1],pts[1:])):
        parts.append((f'cable_{i}',coal.Capsule(.0018,float(np.linalg.norm(b-a))),along(a,b)))
    return parts

class AssemblyChecker:
    def __init__(self, envpath, margin=.010):
        self.base=EnvironmentCoalChecker(envpath); self.env=self.base.environment
        self.model=self.base.model; self.data=self.model.createData(); self.margin=margin
        self.gm=pin.GeometryModel(); self.names=[]; self.kinds=[]; self.pairs=[]
        z=np.load(OUT/'robot_convex_envelopes.npz')
        for link in LINKS:
            frame=self.model.frames[self.model.getFrameId(link)]
            obj=pin.GeometryObject(link,frame.parentJoint, self.model.getFrameId(link),frame.placement,convex(z[link+'_v'],z[link+'_f']))
            self.gm.addGeometryObject(obj);self.names.append(link)
        self.robot_count=len(self.names)
        frame=self.model.frames[self.model.getFrameId('wrist_3_link')]
        for name,geom,T in hardware():
            self.gm.addGeometryObject(pin.GeometryObject(name,frame.parentJoint,self.model.getFrameId('wrist_3_link'),frame.placement*T,geom));self.names.append(name)
        self.moving_count=len(self.names)
        self.world_items=[o for o in self.env['objects'] if o.get('collider',True)]
        for o in self.world_items:
            self.gm.addGeometryObject(pin.GeometryObject('world__'+o['name'],0,0,pin.SE3(np.eye(3),np.array(o['position_m'],float)),coal.Box(*o['dimensions_m']))); self.names.append('world__'+o['name'])
        for a,b in self.base.self_pairs:self.addpair(self.names.index(a),self.names.index(b),'robot_self')
        for i in range(self.robot_count,self.moving_count):
            for j in range(self.robot_count-1):self.addpair(i,j,'hardware_robot')
        for i in range(self.moving_count):
            for j,o in enumerate(self.world_items,self.moving_count):
                if i==0 and o['category']=='table':continue
                self.addpair(i,j,'world')
        self.gd=pin.GeometryData(self.gm)
        for req in self.gd.distanceRequests:
            req.enable_signed_distance=False
            req.abs_err=0.;req.rel_err=0.
            req.gjk_tolerance=1e-9;req.epa_tolerance=1e-9
        self.lower=self.base.lower;self.upper=self.base.upper
        self.n=0
        self.world_mask=np.array([k=='world' for k in self.kinds])
        self.motion_bounds=self.make_motion_bounds()
        self.tool_frame=self.model.frames[self.model.getFrameId('tool0')]

    def make_motion_bounds(self):
        radii=[]
        for geom in self.gm.geometryObjects:
            geom.geometry.computeLocalAABB()
            box=geom.geometry.aabb_local
            corners=np.array([[x,y,z] for x in [box.min_[0],box.max_[0]] for y in [box.min_[1],box.max_[1]] for z in [box.min_[2],box.max_[2]]])
            radius=np.linalg.norm(corners@np.asarray(geom.placement.rotation).T+np.asarray(geom.placement.translation),axis=1).max()
            row=np.zeros(6);j=geom.parentJoint
            while j:
                row[j-1]=radius
                radius+=np.linalg.norm(self.model.jointPlacements[j].translation)
                j=self.model.parents[j]
            radii.append(row)
        result=[]
        for pair in self.gm.collisionPairs:
            a,b=pair.first,pair.second
            ja,jb=self.gm.geometryObjects[a].parentJoint,self.gm.geometryObjects[b].parentJoint
            if ja>jb:a,b=b,a;ja,jb=jb,ja
            # In this serial chain, upstream rotations cancel from self-distance.
            r=radii[b].copy();r[:ja]=0
            result.append(r)
        return np.array(result)

    def addpair(self,a,b,kind):
        self.gm.addCollisionPair(pin.CollisionPair(a,b)); self.kinds.append(kind); self.pairs.append([self.names[a],self.names[b]])

    def evaluate(self,q,details=False):
        q=np.asarray(q,float)
        if not np.isfinite(q).all() or np.any(q<self.lower) or np.any(q>self.upper):return np.array([-np.inf,-np.inf,-np.inf])
        pin.forwardKinematics(self.model,self.data,q)
        pin.updateGeometryPlacements(self.model,self.data,self.gm,self.gd)
        pin.computeDistances(self.gm,self.gd)
        self.n+=1
        ds=np.array([r.min_distance for r in self.gd.distanceResults])
        out=np.array([min(ds[i] for i,k in enumerate(self.kinds) if k==kind) for kind in ['world','robot_self','hardware_robot']])
        if details:return dict(clearance=out.tolist(),minimum_pair=self.pairs[int(np.argmin(ds))],pair_distances=ds.tolist(),pairs=self.pairs)
        return out

    def state(self,q):
        pin.forwardKinematics(self.model,self.data,np.asarray(q,float))
        pin.updateGeometryPlacements(self.model,self.data,self.gm,self.gd)
        pin.computeDistances(self.gm,self.gd);self.n+=1
        ds=np.array([r.min_distance for r in self.gd.distanceResults])
        tf=self.data.oMi[self.tool_frame.parentJoint]*self.tool_frame.placement
        return ds,np.array(tf.translation)

    def edge(self,a,b,da=None,db=None,intervals=16):
        """Skip pairs already proved safe over the WHOLE edge.

        World pairs may be skipped only when their whole-edge bound exceeds
        the 100 mm zero-penalty threshold. Other pairs need exceed 10 mm.
        This is a deterministic lower-bound optimization, not a sampled filter.
        """
        da,pa=self.state(a);db,pb=self.state(b)
        if min(da.min(),db.min())<=self.margin+2e-6:return None
        dq=b-a;motion=self.motion_bounds@abs(dq)
        whole=np.minimum(da,db)-motion/2-2e-6
        disabled=whole>np.where(self.world_mask,.10,self.margin)+2e-6
        for i in np.flatnonzero(disabled):self.gd.deactivateCollisionPair(int(i))
        motion[disabled]=0
        try:
            da[disabled]=whole[disabled];db[disabled]=whole[disabled]
            ds=[da];xyz=[pa]
            for i in range(1,intervals):
                d,p=self.state(a+(i/intervals)*dq);d[disabled]=whole[disabled]
                if d.min()<=self.margin+2e-6:return None
                ds.append(d);xyz.append(p)
            ds.append(db);xyz.append(pb);ds=np.array(ds);xyz=np.array(xyz)
            lower=float('inf');minimum=float(ds.min());extra=0
            stack=[(i/intervals,(i+1)/intervals,ds[i],ds[i+1],0) for i in range(intervals)]
            while stack:
                l,r,dl,dr,depth=stack.pop();bound=np.minimum(dl,dr)-motion*(r-l)/2-2e-6
                if bound.min()>self.margin:lower=min(lower,float(bound.min()));continue
                if depth>=18:return None
                mid=(l+r)/2;dm,_=self.state(a+mid*dq);dm[disabled]=whole[disabled];extra+=1
                minimum=min(minimum,float(dm.min()))
                if dm.min()<=self.margin+2e-6:return None
                stack.extend([(l,mid,dl,dm,depth+1),(mid,r,dm,dr,depth+1)])
            world=ds[:,self.world_mask].min(axis=1);weights=np.asarray(self.env['objectives']['joint_path_weights']);arc=np.linalg.norm(dq*weights)
            costs=np.array([np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum(),np.trapezoid(np.maximum(0,1-world/.10)**2,dx=1/intervals)*arc])
            return costs,np.array([minimum,lower,world.min(),extra])
        finally:
            for i in np.flatnonzero(disabled):self.gd.activateCollisionPair(int(i))

    def edge_reference(self,a,b,da=None,db=None,intervals=16):
        """Conservative all-time clearance certificate, plus uniform cost samples.

        Distance between each convex pair is 1-Lipschitz under object motion.
        Each point is at most half an interval from its nearest endpoint; the
        serial-chain radius bound includes every modeled moving body. Failure
        to prove the bound after 18 bisections rejects, never accepts, an edge.
        """
        dq=b-a; ds=[]; xyz=[]
        for i in range(intervals+1):
            d,p=self.state(a+(i/intervals)*dq)
            if d.min()<=self.margin+2e-6:return None
            ds.append(d);xyz.append(p)
        ds=np.array(ds);xyz=np.array(xyz)
        motion=self.motion_bounds@abs(dq)
        lower=float('inf');minimum=float(ds.min());extra=0
        stack=[(i/intervals,(i+1)/intervals,ds[i],ds[i+1],0) for i in range(intervals)]
        while stack:
            l,r,dl,dr,depth=stack.pop()
            bound=np.minimum(dl,dr)-motion*(r-l)/2-2e-6
            if bound.min()>self.margin:
                lower=min(lower,float(bound.min()));continue
            if depth>=18:return None
            mid=(l+r)/2;dm,_=self.state(a+mid*dq);extra+=1
            minimum=min(minimum,float(dm.min()))
            if dm.min()<=self.margin+2e-6:return None
            stack.extend([(l,mid,dl,dm,depth+1),(mid,r,dm,dr,depth+1)])
        world=ds[:,self.world_mask].min(axis=1)
        weights=np.asarray(self.env['objectives']['joint_path_weights'])
        arc=np.linalg.norm(dq*weights)
        costs=np.array([np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum(),np.trapezoid(np.maximum(0,1-world/.10)**2,dx=1/intervals)*arc])
        return costs,np.array([minimum,lower,world.min(),extra])

    def fk(self,q):return self.base.tool_position(q)

def main():
    reports={}
    bank=np.load(ROOT/'results/one_obstacle_20260907/geometry/AIR/ik_goals.npz')
    print('bank',[(k,bank[k].shape) for k in bank.files],flush=True)
    for alias in ['E4','E7','E5']:
        source=ROOT/'results/trajectory_showcase_5M_wrist_rgbd_20260921'/alias
        c=AssemblyChecker(source/'environment.json')
        q=np.load(source/'02_safest/q_original.npz')['q'][0]
        t=time.monotonic(); start=c.evaluate(q,True)
        gd=np.array([c.evaluate(x) for x in bank['q']])
        r=dict(start=start,valid_goals=int(np.sum(gd.min(axis=1)>.010)),total_goals=len(gd),goal_max_clearance_m=float(gd.min(axis=1).max()),seconds_per_state=(time.monotonic()-t)/(len(gd)+1))
        reports[alias]=r
        print(alias,{k:v for k,v in r.items() if k!='start'},'start',start['clearance'],start['minimum_pair'],flush=True)
    (OUT/'initial_assembly_feasibility.json').write_text(json.dumps(reports,indent=2))

if __name__=='__main__':main()
