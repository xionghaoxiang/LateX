import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import matplotlib.patheffects as pe

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(6, 2.5))

# ---- 图7：导弹视角投影 ----
# 圆柱外轮廓
from matplotlib.patches import Ellipse
ax1.add_patch(Ellipse((0, 0), 3.0, 0.8, lw=1.2, ec='k', fc='none'))   # 下圆
ax1.add_patch(Ellipse((0, 2), 3.0, 0.8, lw=1.2, ec='k', fc='none'))   # 上圆
ax1.plot([-1.5, -1.5], [0, 2], 'k', lw=1.2)
ax1.plot([ 1.5,  1.5], [0, 2], 'k', lw=1.2)
# 烟幕
ax1.add_patch(Ellipse((0, 1), 2.4, 0.7, color='orangered', alpha=0.3))
ax1.text(0, 1, 'Smoke', ha='center', va='center', color='darkred')
ax1.set(xlim=(-2, 2), ylim=(-0.5, 2.7), aspect='equal')
ax1.axis('off')
ax1.set_title('图7：圆柱体被遮蔽示意', fontsize=9)

# ---- 图8：截面 PAB ----
ax2.add_patch(Rectangle((-1.5, 0), 3, 2, ec='k', lw=1.2, fc='lightblue', alpha=0.2))
ax2.plot([-1.5, 1.5], [0, 0], 'k', lw=1.2)
ax2.plot([-1.5, 1.5], [2, 2], 'k', lw=1.2)
ax2.add_patch(Circle((0, 1), 1.2, color='orangered', alpha=0.3))
ax2.text(0, 1, 'Smoke', ha='center', va='center', color='darkred')
ax2.set(xlim=(-2, 2), ylim=(-0.3, 2.3), aspect='equal')
ax2.axis('off')
ax2.set_title('图8：截面 PAB 示意', fontsize=9)

plt.tight_layout()
plt.savefig('cylinder_occlusion_demo.pdf', bbox_inches='tight')
plt.show()