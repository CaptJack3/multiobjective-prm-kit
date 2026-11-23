"""Shared exact/A*pex kernel; costs and parent chains are immutable.

Approximate labels represent multiple histories. Therefore only exact mode
may prune a successor using the representative's ancestry. Closed pairs are
reopened whenever their apex decreases. A terminal pair additionally owns
the lower bounds of every branch it epsilon-pruned (A*pex Algorithm 2,
lines 21--22); those obligations must survive every later replacement.
"""
import heapq
import numpy as np
from numba import njit

MERGE_ATOL = 1e-12  # Historical internal tolerance; external audit is 1e-9.

@njit(cache=True)
def weak(a,b):
    return a[0]<=b[0] and a[1]<=b[1] and a[2]<=b[2]

@njit(cache=True)
def strict(a,b):
    return weak(a,b) and (a[0]<b[0] or a[1]<b[1] or a[2]<b[2])

@njit(cache=True)
def covers(a,b,eps):
    # No division: zero has only the explicitly documented absolute tolerance.
    return a[0]<=(1+eps)*b[0]+MERGE_ATOL and a[1]<=(1+eps)*b[1]+MERGE_ATOL and a[2]<=(1+eps)*b[2]+MERGE_ATOL

@njit(cache=True)
def advance(indptr,nbr,eids,ec,h,sg,eps,heap,solutions,head,node,parent,edge,g,apex,nxt,status,count,counters,steps):
    expanded=0;cap=len(node)
    while len(heap) and expanded<steps:
        _,_,_,lid=heapq.heappop(heap)
        if status[lid]!=1:continue
        status[lid]=2;expanded+=1;counters[1]+=1;u=node[lid]
        if count+2*(indptr[u+1]-indptr[u])+2>=cap:return count,True
        if u==sg:
            dominated=False
            for sol in solutions:
                if status[sol]==3 and strict(g[sol],g[lid]):
                    if eps>0:apex[sol]=np.minimum(apex[sol],apex[lid])
                    dominated=True;break
            if not dominated:
                for sol in solutions:
                    if status[sol]==3 and strict(g[lid],g[sol]):
                        if eps>0:apex[lid]=np.minimum(apex[lid],apex[sol])
                        status[sol]=4
                status[lid]=3;solutions.append(lid)
            continue
        for ai in range(indptr[u],indptr[u+1]):
            v=nbr[ai];eid=eids[ai];counters[0]+=1
            if eps==0:
                anc=lid;cycle=False
                while anc>=0:
                    if node[anc]==v:cycle=True;break
                    anc=parent[anc]
                if cycle:continue
            cg=g[lid]+ec[eid];ca=apex[lid]+ec[eid];cp=lid;ce=eid
            if v==sg:
                node[count]=v;parent[count]=cp;edge[count]=ce;g[count]=cg;apex[count]=ca;status[count]=5;nxt[count]=-1;count+=1
            prune=False
            for sol in solutions:
                if status[sol]!=3:continue
                if (eps>0 and covers(g[sol],ca+h[v],eps)) or (eps==0 and strict(g[sol],cg+h[v])):
                    if eps>0:apex[sol]=np.minimum(apex[sol],ca+h[v])
                    prune=True;counters[2]+=1;break
            if prune:continue
            other=head[v]
            while other>=0:
                if status[other] in (1,2,3):
                    if eps==0:
                        if strict(g[other],cg):prune=True;counters[3]+=1;break
                    else:
                        if weak(apex[other],ca) and weak(g[other],cg):
                            prune=True;counters[3]+=1;break
                        merged=np.minimum(ca,apex[other]);mf=merged+h[v]
                        c_ok=covers(cg+h[v],mf,eps);o_ok=covers(g[other]+h[v],mf,eps)
                        counters[4]+=1
                        if c_ok or o_ok:
                            if o_ok and (not c_ok or np.sum(g[other])<np.sum(cg)):
                                if weak(apex[other],merged):
                                    prune=True;counters[3]+=1;break
                                cg=g[other].copy();cp=parent[other];ce=edge[other]
                            ca=merged;status[other]=4;counters[5]+=1
                other=nxt[other]
            if prune:continue
            other=head[v]
            while other>=0:
                if status[other] in (1,2) and strict(cg,g[other]) and (eps==0 or weak(ca,apex[other])):
                    status[other]=4;counters[3]+=1
                other=nxt[other]
            node[count]=v;parent[count]=cp;edge[count]=ce;g[count]=cg;apex[count]=ca;status[count]=1
            nxt[count]=head[v];head[v]=count;f=ca+h[v]
            heapq.heappush(heap,(f[0],f[1],f[2],count));count+=1
            counters[6]=max(counters[6],len(heap))
            if count>=cap-2:return count,True
    return count,False
