import matplotlib.pyplot as plt
import matplotlib
import matplotlib.ticker as mtick
from mpl_toolkits.mplot3d import Axes3D
import sys
sys.path.insert(0, '/home/tom/Projects/SLAM/post_processing')
from load_data import loadTrajectory
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from matplotlib.lines import Line2D

plt.rcParams['text.usetex'] = True #Let TeX do the typsetting
plt.rcParams['text.latex.preamble'] = [r'\usepackage{sansmath}', r'\sansmath'] #Force sans-serif math mode (for axes labels)
plt.rcParams['font.family'] = 'sans-serif' # ... for regular text
plt.rcParams['font.sans-serif'] = 'Helvetica, Avant Garde, Computer Modern Sans serif' # Choose a nice font here

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]


def draw_gps_heading(trajectory, ax, scale=0.5, marker='o'):

	t = trajectory[['Easting','Northing','height']]
	heading = trajectory[['HEADING']]
	heading = np.deg2rad(float(heading))

	ax.scatter(t[0],t[1], marker=marker, c='tab:blue')
	arm = [t[0] + scale*np.sin(heading), t[1] + scale*np.cos(heading)]
	ax.plot([t[0], arm[0]],
			 [t[1], arm[1]], 'k--')
	return ax


def draw_slam_heading(trajectory, ax, scale=0.5, marker='o'):

	t = trajectory[['x','y','z','qx','qy','qz','qw']]

	ax.scatter(t[2],-t[0], marker=marker, c='tab:orange')

	[yaw, pitch, roll] = quaternion_to_euler(t['qx'],t['qy'],t['qz'],t['qw'])
	angle = pitch
	arm = [t[0] + scale*np.sin(angle),0, t[2] + scale*np.cos(angle)]
	ax.plot([t[2], arm[2]],
			 [-t[0], -arm[0]], 'k--')

	return ax


def draw_edge(p1, p2, ax):
	ax.plot3D([p1[0], p2[0]],
			  [p1[1], p2[1]],
			  [p1[2], p2[2]], c='k--')
	return ax

def draw_gps_camera(trajectory, ax, scale=1):

	# unit camera
	c = scale*np.array([0,0,0])
	tl = scale*np.array([-1,1,1])
	tr = scale*np.array([1,1,1])
	bl = scale*np.array([-1,1,-1])
	br = scale*np.array([1,1,-1])

	# transform by camera
	t = trajectory[['Easting','Northing','height']]
	heading = trajectory[['HEADING']]

	Rot = R.from_euler('xyz', [0,0,90-heading], degrees=True)

	c = Rot.apply(c) + t
	tr = Rot.apply(tr) + t
	tl = Rot.apply(tl) + t
	bl = Rot.apply(bl) + t
	br = Rot.apply(br) + t

	# camera vertices
	ax.scatter(c[0],c[1],c[2], s=40, c='r')
	ax.scatter(tl[0],tl[1],tl[2],c='k')
	ax.scatter(tr[0],tr[1],tr[2],c='k')
	ax.scatter(bl[0],bl[1],bl[2],c='k')
	ax.scatter(br[0],br[1],br[2],c='k')
	# camera edges
	ax = draw_edge(tl,tr,ax)
	ax = draw_edge(tl,bl,ax)
	ax = draw_edge(tr,br,ax)
	ax = draw_edge(bl,br,ax)
	ax = draw_edge(c,tr,ax)
	ax = draw_edge(c,tl,ax)
	ax = draw_edge(c,br,ax)
	ax = draw_edge(c,bl,ax)

	return ax

def main():

	trajectories = loadTrajectory()

	# plotting gps
	#fig, ax = plt.subplots(2,3)
	#ax.set_aspect('equal')
	#ax2.set_aspect('equal')
	indices1 = [(0,100,200,5,8,0.8,'o')]
	indices2 = [(1,2550,2650,5,8,0.8,'o')]
	indices3 = [(2,950,1050,5,8,0.8,'o')]
	indices4 = [(3,200,3500,20,0,0,'.')]

	#indices = indices4
	indices = indices1 + indices2 + indices3 + indices4

	for (ind, start, stop, step, scale_gps, scale_slam, marker) in indices:

		fig = plt.figure()

		ax1=fig.add_subplot(111, label="1")
		ax2=fig.add_subplot(111, label="2", frame_on=False)

		trajectories_plot = trajectories.iloc[start:stop:step]
		for i, trajectory in trajectories_plot.iterrows():
			# gps
			ax1 = draw_gps_heading(trajectory, ax1, scale=scale_gps, marker=marker)
			# slam
			ax2 = draw_slam_heading(trajectory, ax2, scale=scale_slam, marker=marker)

		ax2.xaxis.tick_top()
		ax2.yaxis.tick_right()
		ax2.set_xlabel('z [m]', color="C1")
		ax2.set_ylabel('x [m]', color="C1")
		ax2.xaxis.set_label_position('top')
		ax2.yaxis.set_label_position('right')
		ax2.tick_params(axis='x', colors="C1")
		ax2.tick_params(axis='y', colors="C1")

		ax1.set_xlabel("Easting [m]", color="C0")
		ax1.set_ylabel("Northing [m]", color="C0")
		ax1.tick_params(axis='x', colors="C0")
		ax1.tick_params(axis='y', colors="C0")

		if ind < 3:
			custom_lines = [Line2D([0], [0], marker='o', color='w', markerfacecolor='tab:blue', lw=4, label='GPS-enabled IMU'),
	               			Line2D([0], [0],  marker='o', color='w', markerfacecolor='tab:orange', lw=4, label='Modified ORB-SLAM'),
							Line2D([0], [0],  color='k', linestyle='--', lw=4, label='Direction')]

		else:
			custom_lines = [Line2D([0], [0], marker='o', color='w', markerfacecolor='tab:blue', lw=4, label='GPS-enabled IMU'),
	               			Line2D([0], [0],  marker='o', color='w', markerfacecolor='tab:orange', lw=4, label='Modified ORB-SLAM')]

		plt.legend(handles=custom_lines)
		#plt.tight_layout()
		plt.savefig("slam_v_gps_{}_type1.pdf".format(ind))
		#plt.show()
			#ax1.set_aspect('equal')

	#ax[0][1].set_xlabel('Easting [m]')
	#ax[1][1].set_xlabel('z [m]')

	#ax[0][0].set_ylabel('Northing [m]')
	#ax[1][0].set_ylabel('x [m]')
	#ax[1][1].yaxis.set_label_position("right")
	#ax[1][1].yaxis.tick_right()

	#ax[0][1].yaxis.set_label_position("right")
	#ax[0][1].yaxis.tick_right()
	#ax[2][1].yaxis.set_label_position("right")
	#ax[2][1].yaxis.tick_right()



if __name__ == '__main__':
	main()
