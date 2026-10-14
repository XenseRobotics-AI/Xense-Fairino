from pathlib import Path
import zipfile,struct,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.collections import PolyCollection
from matplotlib.font_manager import FontProperties
root=Path('/home/lim/Documents/法奥/lidar_diagnostics/model_geometry')
archive=Path('/home/lim/Documents/法奥/轮足机器人URDF模型外发-v1.0-20260731.zip')
with zipfile.ZipFile(archive) as z:
 name='fairino_wheel_legged_robot_description/meshes/base_link.STL';data=z.read(name)
num=struct.unpack_from('<I',data,80)[0];assert len(data)==84+50*num
records=np.frombuffer(data,offset=84,dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]))
v=records['vertices'].astype(float);n=records['normal'].astype(float)
np.savez(root/'base_mesh.npz',triangles=v,normals=n)
verts=np.unique(v.reshape(-1,3),axis=0)
lo=verts.min(0);hi=verts.max(0)
zvals,zcnt=np.unique(np.round(verts[:,2],5),return_counts=True)
print(json.dumps({'triangles':num,'unique_vertices':len(verts),'bounds_m':[lo.tolist(),hi.tolist()],'frequent_z_m':[[float(zvals[i]),int(zcnt[i])] for i in np.argsort(zcnt)[-20:][::-1]]}))
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False})
fig=plt.figure(figsize=(13,7));ax=fig.add_subplot(121,projection='3d')
light=np.array([.4,-.5,.8]);light/=np.linalg.norm(light);shade=.55+.4*np.abs(n@light);colors=np.c_[shade*.88,shade*.92,shade*.96,np.ones(num)]
col=Poly3DCollection(v,facecolors=colors,edgecolor=(.1,.15,.2,.25),linewidth=.08)
ax.add_collection3d(col);ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_zlim(lo[2],hi[2]);ax.set_box_aspect(hi-lo);ax.view_init(elev=30,azim=-60);ax.set_xlabel('模型 X / m');ax.set_ylabel('模型 Y / m');ax.set_zlabel('模型 Z / m');ax.set_title('URDF 引用的底盘整体网格')
ax2=fig.add_subplot(122)
order=np.argsort(v[:,:,2].mean(1));c=plt.cm.viridis((v[:,:,2].mean(1)-lo[2])/(hi[2]-lo[2]))
ax2.add_collection(PolyCollection(v[order,:,:2],facecolors=c[order],edgecolors=(0,0,0,.15),linewidth=.1));ax2.set_xlim(lo[0]-.03,hi[0]+.03);ax2.set_ylim(lo[1]-.03,hi[1]+.03);ax2.set_aspect('equal');ax2.set_xlabel('模型 X / m');ax2.set_ylabel('模型 Y / m');ax2.set_title('俯视图（颜色表示高度）');ax2.grid(alpha=.2)
fig.suptitle('本地 20260731 模型的 base_link.STL',fontsize=16);fig.tight_layout();fig.savefig(root/'base-mesh.png',dpi=150)
