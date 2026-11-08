import sys, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import cadquery as cq
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import agarradera as ag
def tris(shape):
    v, f = shape.val().tessellate(0.05, 0.2)
    v = np.array([(p.x, p.y, p.z) for p in v]); return v[np.array(f)]
parts = [(tris(ag.carcasa()), np.array([0.93,0.89,0.78])), (tris(ag.incruste()), np.array([0.15,0.15,0.15]))]
light = np.array([0.4,-0.6,0.8]); light/=np.linalg.norm(light)
views = [("iso1",25,-60),("iso2",30,120),("arriba",90,-90),("punta",10,80)]
fig = plt.figure(figsize=(12,10))
for k,(name,el,az) in enumerate(views):
    ax = fig.add_subplot(2,2,k+1,projection="3d")
    TT=[];CC=[]
    for T,col in parts:
        n = np.cross(T[:,1]-T[:,0], T[:,2]-T[:,0]); n/= (np.linalg.norm(n,axis=1)[:,None]+1e-9)
        sh = 0.35+0.65*np.abs(n@light); TT.append(T); CC.append(np.clip(col*sh[:,None],0,1))
    ax.add_collection3d(Poly3DCollection(np.concatenate(TT), facecolors=np.concatenate(CC), edgecolors="none"))
    ax.set_xlim(-3,50); ax.set_ylim(-3,56); ax.set_zlim(-3,42); ax.set_box_aspect((53,59,45))
    ax.view_init(el,az); ax.set_title(name); ax.set_xlabel("X (A=47)"); ax.set_ylabel("Y (B=52.9)")
plt.tight_layout(); plt.savefig(sys.argv[1], dpi=110)
