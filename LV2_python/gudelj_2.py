import sys
import numpy as np
import vtk
import matplotlib.pyplot as plt
import vtk_visualizer as vis
#import br_lectures as br
from hocook import hocook
from planecontact import planecontact
from mobrobsim import mobrobsimanimate, set_goal, set_map
from scipy import ndimage
from PIL import Image
from camerasim import CameraSimulator
#from skimage import feature
#from skimage.transform import hough_line, hough_line_peaks

# TASK 0
class tool():
	def __init__(self, scene):
		s = scene	
		self.finger1 = vis.cube(0.04, 0.06, 0.08)
		s.add_actor(self.finger1)
		self.finger2 = vis.cube(0.04, 0.06, 0.08)
		s.add_actor(self.finger2)
		self.palm = vis.cube(0.04, 0.25, 0.02)
		s.add_actor(self.palm)
		self.wrist = vis.cylinder(0.015, 0.04)
		s.add_actor(self.wrist)
		
	def set_configuration(self, g, TGS):	
		TF1G = np.identity(4)
		TF1G[:3,3] = np.array([0, -0.095, -0.03])
		TF1S = TGS @ TF1G
		vis.set_pose(self.finger1, TF1S)
		TF2G = np.identity(4)
		TF2G[:3,3] = np.array([0, 0.095, -0.03])	
		TF2S = TGS @ TF2G
		vis.set_pose(self.finger2, TF2S)
		TPG = np.identity(4)
		TPG[:3,3] = np.array([0, 0, -0.065])
		TPS = TGS @ TPG
		vis.set_pose(self.palm, TPS)
		TWG = np.block([[rotx(np.pi/2), np.array([[0], [0], [-0.08]])], [np.zeros((1, 3)), 1]])
		TWS = TGS @ TWG
		vis.set_pose(self.wrist, TWS)

def rotx(q):
	c = np.cos(q)
	s = np.sin(q)
	return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
	
def roty(q):
	c = np.cos(q)
	s = np.sin(q)
	return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rotz(q):
	c = np.cos(q)
	s = np.sin(q)
	return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def set_floor(s, size):
	floor = vis.cube(size[0], size[1], 0.01)
	TFS = np.identity(4)
	TFS[2,3] = -0.005
	vis.set_pose(floor, TFS)
	s.add_actor(floor)

def task0():
	# Scene.
	s = vis.visualizer()

	# Axes.
	#axes = vtk.vtkAxesActor()
	#s.add_actor(axes)
	
	# Floor.
	set_floor(s, [1, 1])

	# Cube.
	cube = vis.cube(0.03, 0.03, 0.03)
	TCS = np.identity(4)
	TCS[:3,3] = np.array([0, 0, 1.5])
	vis.set_pose(cube, TCS)
	s.add_actor(cube)
	
	# Tool.
	TGS = np.identity(4)
	TGS[:3,3] = np.array([0, 0, 1.0])
	tool_ = tool(s)
	tool_.set_configuration(0.015, TGS)
		
	# Render scene.
	s.run()


# TASK 1
class robot():
	def __init__(self, scene):
		#kinematic parameters:
		q = np.array([np.pi/2, -np.pi/2, np.pi/2, 0, 0, -np.pi/2])
		d = np.array([0, 0, 0, 0.790, 0, 0.205])
		a = np.array([0, 0.79 , 0, 0, 0, 0])
		al = np.array([-np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0])

		self.DH = np.stack((q, d, a, al), 1)
	
		s = scene

		
		# Base.
		#self.base = vis.cylinder(0.025, 0.05)
		self.base = vis.cube(0.343, 0.343, 0.2625)
		s.add_actor(self.base)	

		# Link 1.
		#self.link1 = vis.cylinder(0.025, 0.05)
		self.link1 = vis.cube(0.24, 0.3625 , 0.24)
		s.add_actor(self.link1)	
		
		#Link 2
		self.link2 = vis.cube(0.990,0.150 ,0.138)
		s.add_actor(self.link2)

		#Link 3
		self.link3 = vis.cube(0.24, 0.24, 0.23)
		s.add_actor(self.link3)

		# Link 4.
		self.link4 = vis.cube(0.15, 0.790,0.15)
		s.add_actor(self.link4)	
		
		# Link 5.
		self.link5 = vis.cylinder(0.075, 0.150)
		s.add_actor(self.link5)		
		
		# Tool.
		self.tool = tool(s)
		
		#Cube
		#self.link6 = vis.cube(0.13,0.13,0.13)
		#self.link6.GetProperty().SetColor(1,0,0)
		#s.add_actor(self.link6)		


		
		

		
	def set_configuration(self, q, g, T0S):
		d = self.DH[:,1]
		a = self.DH[:,2]
		al = self.DH[:,3]
		
		# Base.
		TB0 = np.identity(4)
		TB0[2,3] = -0.39375
		TBS = T0S @ TB0
		vis.set_pose(self.base, TBS)

		# Link 1.
		T10 = dh(q[0], d[0], a[0], al[0]) 
		T1S = T0S @ T10
		#Rotacijska 1
		TL11 = np.identity(4)
		TL11[1,3] = 0.08125
		TL1S = T1S @ TL11
		vis.set_pose(self.link1, TL1S)
		
		#Link 2
		T21 = dh(q[1], d[1], a[1], al[1])
		T2S = T1S @ T21	
		#Rotacijska 2
		TL22 = np.identity(4)
		TL22[0,3] = -0.495
		TL22[2,3] = 0.18917
		TL2S = T2S @ TL22
		vis.set_pose(self.link2, TL2S)

		#Link 3
		T32 = dh(q[2], d[2], a[2], al[2]) 
		T3S = T2S @ T32
		#Rotacijska 3
		TL33 = np.identity(4)
		T3LS = T3S @ TL33
		vis.set_pose(self.link3, T3LS)

		# Link 4.
		T43 = dh(q[3], d[3], a[3], al[3])
		T4S = T3S @ T43	
		#Rotacijska 4
		TL44 = np.identity(4)
		TL44[1,3]=0.395
		TL4S = T4S @ TL44
		vis.set_pose(self.link4, TL4S)
		
		# Link 5.
		T54 = dh(q[4], d[4], a[4], al[4])
		T5S = T4S @ T54	
		#Rotacijska 5
		TL55 = np.identity(4)
		TL5S = T5S @ TL55
		vis.set_pose(self.link5, TL5S)
		
		# Link 6.
		T65 = dh(q[5], d[5], a[5], al[5])
		T6S = T5S @ T65	
		#Rotacijska 6
		TL66 = np.identity(4)
		TL66[2,3]=-0.05
		TL6S = T6S @ TL66
		self.tool.set_configuration(g, TL6S)

		#Cube

		#TLC = np.identity(4)
		#TLC[2,3] = -0.045 
		#TLCS = T6S @ TLC
		#vis.set_pose(self.link6, TLCS)

		return T6S	

	

def dh(q, d, a, al):

	#Define cq, sq, ca, sa and T
	cq = np.cos(q)
	sq = np.sin(q)
	ca = np.cos(al)
	sa = np.sin(al)
	T = np.array([[cq, -sq*ca, sq*sa, a*cq],
		[sq, cq*ca, -cq*sa, a*sq],
		[0, sa, ca, d],
		[0, 0, 0, 1]])
	return T
	return T

def task1(q):
	# Scene.
	s = vis.visualizer()

	# Axes.
	axes = vtk.vtkAxesActor()
	s.add_actor(axes)

	# Floor.
	set_floor(s, [1, 1])

	TTS = np.identity(4)
	TTS[0,3]=	0.3
	TTS[1,3] = -0.12
	TTS[2,3] = 	0.12
	
	
	# Target object.
	target = vis.cube(0.08, 0.08, 0.08) #change size
	vis.set_pose(target, TTS)
	s.add_actor(target)	
	
	# Robot.
	T0S = np.identity(4)
	T0S[2,3] = 0.525
	rob = robot(s)
	rob.set_configuration(q, 0.03, T0S)
	
	# Render scene.
	s.run()	

# TASK 2

def invkin(DH, T60, solution):
	d = DH[:,1]
	a = DH[:,2]
	al = DH[:,3]
	
	p = T60 @ np.expand_dims(np.array([0, 0, -d[5], 1]), 1)
	#p = T60 @ [0,0,-d[5],1]
	x = p[0]
	y = p[1]
	z = p[2]
	r = p[:3].T @ p[:3]
	
	q = np.zeros(6)
	
	q[2] = np.arccos((r - a[2]**2 - d[3]**2 - a[1]**2) / (2*a[1]*a[2]))
	if solution[0] == 1:
		q[2] = -q[2]
		
	c3 = np.cos(q[2])
	s3 = np.sin(q[2])
	f1 = a[2]*c3 + a[1]
	
	f2 = a[2]*s3
	
	f3 = d[3]
	
	A = np.sqrt(f1**2+f2**2)
	phi = np.arctan2(f2, f1)
	if solution[1] == 0:
		q[1] = np.arcsin(z/A) - phi
	else:
		q[1] = np.pi - np.arcsin(z/A) - phi
	

	c2 = np.cos(q[1])
	s2 = np.sin(q[1])
	g1 = c2*f1 - s2*f2
		
	g2 = -f3
	
	c1 = g1*x + g2*y
	s1 = -g2*x + g1*y
	q[0] = np.arctan2(s1, c1)
	
	T10 = dh(q[0], d[0], a[0], al[0])
	T21 = dh(q[1], d[1], a[1], al[1])
	T32 = dh(q[2], d[2], a[2], al[2])
	T30 = T10 @ T21 @ T32
	R30 = T30[:3,:3]
	R60 = T60[:3,:3]
	R63 = R30.T @ R60
	
	c5 = -R63[2,2]
	q[4] = np.arccos(c5)
	if solution[2] == 1:
		q[4] = -q[4]
	s5 = np.sin(q[4])
	if np.abs(s5) > 1e-10:
		q[3] = np.arctan2(R63[1,2]/s5, R63[0,2]/s5)
		q[5] = np.arctan2(-R63[2,1]/s5, R63[2,0]/s5)
	else:
		c46 = R63[0,0]
		s46 = R63[0,1]
		q46 = np.arctan2(s46, c46)
		q[3] = q46
		q[5] = 0
	
	return q

def task2(solution):
	TTS = np.identity(4)
	TTS[0,3]=	0.3
	TTS[2,3] = 	0.1
	
	# Scene
	s = vis.visualizer()
	axes = vtk.vtkAxesActor()
	s.add_actor(axes)
	# Floor.
	set_floor(s, [1, 1])
	
	# Target object.
	target = vis.cube(0.08, 0.08, 0.08) #change size
	vis.set_pose(target, TTS)
	s.add_actor(target)	

	# Robot.
	T0S = np.identity(4)
	T0S[2,3] = 0.525
	#T6T = np.identity(4)
	#T6T[:3,:3] = roty(np.pi)
	#T60 = np.linalg.inv(T0S) @ TTS @ T6T

	TA0 = [[1,0,0,0.3],[0,1,0,-0.12],[0,0,1,-0.13125],[0,0,0,1]]
	T6A = [[1,0,0,0],[0,-1,0,0],[0,0,-1,-0.04],[0,0,0,1]]
	T60 = np.matmul(TA0,T6A)
	rob = robot(s)
	#rob.set_configuration([np.pi/2, -np.pi/2, np.pi/2, np.pi/2, 0, 0], 0.03, T0S)
	#s.run()
	q = invkin(rob.DH, T60, solution)
	#q= [-0.99603,1.4353,0.8261,0,0,0]
	rob.set_configuration(q, 0.03, T0S)

	
	# Render scene.
	s.run()

def main():
	#task0()
	#task1([np.pi/2, -np.pi/2, np.pi/2, np.pi/2, 0, 0])
	task2([0, 1, 0])


if __name__ == '__main__':
    main()