"""Inspect CAD lidar housing centers; these are not calibrated scan TF origins."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.font_manager import FontProperties

root = Path(__file__).resolve().parent
rings = json.loads((root / 'circle-analysis.json').read_text())
centers = {}
for label in ('A', 'B'):
    good = [r for r in rings if r['body'] == label
            and r['circle_fit_rms_m'] < 1e-6
            and r['ring_points'] >= 14 and r['radius_m'] > .025]
    assert len(good) >= 2
    centers[label] = np.median([r['center_xy_m'] for r in good], axis=0)
distance = float(np.linalg.norm(centers['B'] - centers['A']))
summary = {
    'source_model': 'models/whole_body_20260731.urdf',
    'urdf_member': 'fairino_wheel_legged_robot_description/urdf/fairino_wheel_legged_robot_description.urdf',
    'mesh_member': 'fairino_wheel_legged_robot_description/meshes/base_link.STL',
    'coordinate_frame': 'base_link of the 20260731 local model',
    'mesh_origin_xyz_m': [0, 0, 0],
    'mesh_origin_rpy_rad': [0, 0, 0],
    'method': 'Circular housing ring fits in XY, two consistent rings per housing',
    'housing_axis_xy_m': {k: v.tolist() for k, v in centers.items()},
    'horizontal_housing_axis_separation_m': distance,
    'previous_scan_registration_separation_m': .6070356767367637,
    'limitations': [
        'Mechanical mesh positions, not measured physical positions or calibrated scan frames.',
        'Labels A/B do not assign physical front/back or IP addresses.',
        'Scan origin height, zero-ray direction and roll/pitch are not established.',
        'Model base_link must not be assumed identical to the running robot base_link.'
    ]
}
(root / 'lidar-centers.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')

mesh = Path(__file__).resolve().parents[3] / 'models/meshes/base_link.STL'
dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
triangles = np.frombuffer(mesh.read_bytes(), offset=84, dtype=dtype)['vertices'].astype(float)
font = (FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc') if Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').exists() else FontProperties(family='DejaVu Sans'))
plt.rcParams.update({'font.family': font.get_name(), 'axes.unicode_minus': False})
fig, ax = plt.subplots(figsize=(9, 10))
height = triangles[:, :, 2].mean(axis=1)
order = np.argsort(height)
colors = plt.cm.Greys(.12 + .42 * (height - height.min()) / np.ptp(height))
ax.add_collection(PolyCollection(triangles[order, :, :2], facecolors=colors[order],
                                 edgecolors=(.2, .3, .4, .07), linewidth=.1))
line = np.array([centers['A'], centers['B']])
ax.plot(line[:, 0], line[:, 1], '--', color='#24599c', lw=2)
for label, color, offset in [('A', '#c34427', (35, 28)), ('B', '#0e726d', (-205, -50))]:
    x, y = centers[label]
    ax.scatter(x, y, color=color, marker='+', s=190, linewidths=2.8, zorder=5)
    ax.add_patch(plt.Circle((x, y), .0312, fill=False, color=color, linewidth=2))
    ax.annotate(f'雷达 {label} 外壳中心轴\nX = {x * 1000:+.0f} mm，Y = {y * 1000:+.0f} mm',
                (x, y), xytext=offset, textcoords='offset points', fontsize=12,
                bbox=dict(boxstyle='round,pad=.45', facecolor='white', edgecolor=color),
                arrowprops=dict(arrowstyle='-', color=color))
ax.text(.006, -.15, f'水平中心距\n{distance * 100:.2f} cm', fontsize=13,
        bbox=dict(boxstyle='round,pad=.45', facecolor='white', edgecolor='#24599c'))
ax.scatter(0, 0, c='#555555', s=30)
ax.annotate('模型原点', (0, 0), xytext=(8, 7), textcoords='offset points', fontsize=10)
ax.set(xlim=(-.335, .335), ylim=(-.565, .265), aspect='equal',
       xlabel='本地模型 base_link 的 X / m', ylabel='本地模型 base_link 的 Y / m',
       title='20260731 本地 URDF 配套底盘网格：雷达机械位置')
ax.grid(alpha=.16)
fig.text(.5, .025, 'A/B 为图中编号；前后与 IP 对应未在此模型中标注。\n外壳中心轴不等同于已经标定的激光坐标原点。',
         ha='center', va='bottom', fontsize=11, color='#444444')
fig.tight_layout(rect=(0, .075, 1, 1))
fig.savefig(root / 'lidar-centers.png', dpi=160)
print(json.dumps({'centers_xy_m': summary['housing_axis_xy_m'], 'separation_m': distance}))
