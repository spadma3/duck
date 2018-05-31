import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

######## README ########
# TODO: scaling is wrong AF. I just did it that way we see something in plot
# TODO: we need to edit the absolute distance function in visual_odometry.py
# TODO: "initial driving orientation" in plot should be downwards. If we stay still,
# data is bullshit (starts to drive in any direction) -> we should start using VO
# as soon as we drive, not before
# TODO: I think this only works if openCV v3+ and not Duckiebots version v2.5
# Maybe we should try to use older functions, or upgrade, I am not sure..
########################
from visual_odometry import PinholeCamera, VisualOdometry

# Measure time
ms_start = int(round(time.time() * 1000))

# Sleep time for debugging
sleeptime = 0.01#0.3
plotting = True

# Cam: width, height, focal length x, focal length y, cam center x, cam center y
cam = PinholeCamera(640.0, 480.0, 1192, 1192, 320.0, 240.0)

# VO: cam, speed
vo = VisualOdometry(cam, 0.1*10)

# Some trajectory or so
traj = np.zeros((600,600,3), dtype=np.uint8)

vec3d = np.array([[0,0,0], [0,0,0]])

t = 2
# Loop through every image (gray scale) in our DB
for img_id in range(22,85):
	# Load image
	img = cv2.imread('DB_duckiebot/test_g_'+str(img_id).zfill(6)+'.png', 0)

	# Update odometry
	vo.update(img)
	# Obtain positions
	cur_t = vo.cur_t
	t = t - 1
	if t >= 0: continue
	if(img_id > 22):
		x, y, z = cur_t[0], cur_t[1], cur_t[2]

		a = -np.pi/36
		vec = np.array([x,y,z])
		R = np.matrix([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
		vec_rot = R.dot(vec)
		x,y,z = float(vec_rot[0]),float(vec_rot[1]),float(vec_rot[2])

	else:
		x, y, z = 0., 0., 0.
	if x != 0:
		vec = np.array([x,y,z])
		vec3d = np.vstack((vec3d,vec))

	# Obtain drawing pixels
	draw_x, draw_y = -int(x*70)+290, int(z*70)+290
	# Draw
	if plotting:
		cv2.circle(traj, (draw_x,draw_y), 1, (img_id*255/4540,255-img_id*255/4500,0), 1)
		cv2.rectangle(traj, (10, 20), (600, 60), (0,0,0), -1)
		text = "Coordinates: x=%2fm y=%2fm z=%2fm"%(x,y,z)
		cv2.putText(traj, text, (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)

		cv2.imshow('Road facing camera', img)
		cv2.imshow('Trajectory', traj)
		cv2.waitKey(1)
		time.sleep(sleeptime)
cv2.waitKey(1)
# Measure time
ms_end =  int(round(time.time() * 1000))
print("Process took " + str(ms_end - ms_start) + "ms with a sleeptime " + str(sleeptime) + "s and plotting " + str(plotting))
cv2.imwrite('map.png', traj)

vec3d = np.delete(vec3d,0,0)
vec3d = np.delete(vec3d,0,0)
shift = vec3d[0]
vec3d = vec3d-shift
fig = plt.figure()
ax = Axes3D(fig)
ax.set_xlim([-1,1])
ax.set_ylim([-1,1])
ax.set_zlim([-1,1])
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.scatter(vec3d[:,0],vec3d[:,1],vec3d[:,2])
plt.show()

time.sleep(1000)
