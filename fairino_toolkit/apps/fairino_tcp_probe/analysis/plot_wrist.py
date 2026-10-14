from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
root=Path('/home/lim/Documents/法奥/法奥轮式人形')
base=next(root.glob('RD53*/left_arm_description/meshes'))
dtype=np.dtype([('normal','<f4',(3,)),('verts','<f4',(3,3)),('attr','<u2')])
fig=plt.figure(figsize=(13,8))
for num,(elev,azim) in enumerate(((20,-60),(90,-90),(0,-90),(0,0)),1):
 ax=fig.add_subplot(2,2,num,projection='3d')
 for name,color in (('wrist1_Link.STL','#69a9d2'),('wrist2_Link.STL','#ec9638')):
  data=np.fromfile(base/name,dtype=dtype,offset=84);tri=data['verts']*1000
  mesh=Poly3DCollection(tri,facecolors=color,edgecolors='none',alpha=.62)
  ax.add_collection3d(mesh)
 for v,c,label in (([45,0,0],'#ce2636','X'),([0,45,0],'#239657','Y'),([0,0,70],'#253fbb','Z / URDF J7 axis')):
  ax.quiver(0,0,0,*v,color=c,linewidth=2);ax.text(*v,label,color=c,fontsize=9)
 ax.scatter([0,-106],[0,0],[0,0],c=['red','black'],s=30)
 ax.text(-110,0,0,'x=-106 mm',fontsize=9)
 ax.set_xlim(-120,55);ax.set_ylim(-60,60);ax.set_zlim(-65,80)
 ax.set_box_aspect((175,120,145));ax.view_init(elev,azim)
 ax.set_xlabel('X (mm)');ax.set_ylabel('Y (mm)');ax.set_zlabel('Z (mm)')
 ax.set_title('URDF wrist frames at J7 = 0')
fig.suptitle('Blue: wrist1 mesh | Orange: wrist2 mesh | Red dot: current DH/URDF end origin',fontsize=13)
fig.tight_layout()
p=Path(__file__).parent/'wrist_geometry.png';fig.savefig(p,dpi=150);print(p)
