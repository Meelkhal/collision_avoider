import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

"""
Module for definitions of objects like DebrisObject, ActiveObject
and superclass PhysicalObject (not just definitions, functions too)
"""

def distance(xyz1, xyz2):
    return np.linalg.norm(xyz1-xyz2)

class PhysicalObject():
    """
    General class for objects with physical properties,
    e.g. debris and active object
    """
    def __init__(self, position_xyz,time,targets,obj):
        # save initial position if needed later
        self._init_x, self._init_y, self._init_z = position_xyz
        # set current coordinates
        self._x, self._y, self._z = position_xyz
        # store position as np array for quicker operations later
        self.position_xyz = np.array(position_xyz)
		self.targets = targets
		self.A = self.targets[0]
		self.B = self.targets{1}
		self._velocity_x = (B[0]-A[0])/10000
		self._velocity_y = (B[1]-A[1])/10000
		self._velocity_z = (B[2]-A[2])/10000
		self.velocity_xyz = np.array(self._velocity_x,self._velocity_y,self._velocity_z)
		initialvelo = [self._vx,self._vy,self_.vz]
	
		# A history of the relevant position,velocity and accelerations will be kept in lists
		self.positions = [position_xyz]
		self.velocities = [initialvelo]
		self.accelerations = [0]
		self.times = [time]
		# these initial velocities allow our obect to drift from A to B initially
		self.obj = obj
		# allows us to distinguish whether the object is a rocket or an asteriod ( might not be useful)
		# obj = 1 for rockets and obj = 0 for asteroids
		if obj == 1:
			self.c_n = [0]
		else:
			# have it give a random array of values for now
			self.c_n = [0,0.02,0.04]
		

	
    def print_position(self):
        print(self.position_xyz)

	# a_nd: acceleration non deterministic, corresponds to sudden random shifts due to random events (faulty rockets) and unknown gravitational pull from faraway objects
	def a_nd(self):
		ep = 10 ** (-2)
		a_prime = [ep*randint(0,100),ep*randint(0,100),ep*randint(0,100)]
		return a_prime
	
	# given taylor series coefficients describes the path of the asteroid as a polynomial in time
	def a_path(self,t): 
		# accleleraiton at a specific time will be given as a taylor series centered around 0 or some explicit term 
		a = 0
		for i in range(len(c_n)):
			a += 1/fact(i) * self.c_n[i] * t**i 
		return a

	def a_impulse(self):
		# Not 100% how this should be handled I think it if in object is detected it should move orthogonal to ray of detection and make a circular arc around that
		return [0,0,0]

	def a_d(self):
		k = 0.05 # minimal drag in space
		a = []
		for i in range(0,3):
			a_i = -k*self.velocity_xyz[i]
		return a

	def acceleration(self,t):
		a = []
		non_deterministic_acceleration = a_nd()
		deterministic_acceleration = a_d()
		impulse_acceleration = a_impulse()
		path_acceleration = a_path(t)
		
		for i in range(0,3):	
			a_i = self.obj*non_deterministic_acceleration[i] + self.obj*deterministic_acceleration[i] + (1-self.obj)*a_path[i]+b*a_impulse[i]
			a.append(a_i)
		return a
		
	def add_vect(self,x,y,a,b):
		z = []
		for i in range(len(x)):
			z.append(a*x[i]+b*y[i])
		return z
	
	def time_update(self,step):
		self.time = self.time+h
		vf = 0
		a = acceleration(self.time)
		vf = add_vect(self.velocities[-1],a,1,step)
		self.velocity_xyz = vf
		# might need to convert these lists to numpy arrays
		rf = add_vect(self.velocities[-1],vf,1,step)
		self.position_xyz = rf
		
		# adds changes to the new velocities 
		self.positions.append(rf)
		self.velocities.append(vf)
		self.velocities.append(a)

		

class DebrisObject(PhysicalObject):
    """
    Class for describing debris objects
    On initialization, gets a well-defined path,
    which will not be visible to active object
    """
    def __init__(self, position_xyz):
        super(DebrisObject, self).__init__(position_xyz)
        # calculate path somehow? We'll probably need more info than position
        # TODO: Mujtaba calculate this

class ActiveObject(PhysicalObject):
    """
    Has distances to nearby objects from a few frames in the past
    Has endpoints of trip
    Has fuel which gets depleted based on motion
    Tries to minimize/maximize hitting stuff
    while also minimizing fuel
    """

    def __init__(self, A_xyz, B_xyz, radius=1, fuel=1):
        # radius that we can scan for debris
        self.scan_radius = radius

        # set initial position
        super(ActiveObject, self).__init__(A_xyz)

        # set destination
        self.destination_xyz = B_xyz

        # fuel amount
        self.initial_fuel = fuel
        self.current_fual = fuel

        # nothing is visible yet
        self.visible_debris_positions = None

    def scan(self, debris_list):
        """
            parameters:
                debris_list is a list of DebrisObjects
            returns:
                positions of visible (distance <= radius) debris,
                list of (x,y,z) tuples.
        """
        # look around to see if there is debris nearby
        visible_debris_positions = np.array([
            d.position_xyz for d in debris_list
            if distance(self.position_xyz, d.position_xyz) <= self.scan_radius
        ])
        # set visible_debris_positions
        self.visible_debris_positions = visible_debris_positions

    def view(self, debris_list):
        """make 3d view of active object and debris"""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # plot active object at origin
        ax.scatter(self.position_xyz[0],
                   self.position_xyz[1],
                   self.position_xyz[2],
                   c='g', s=5)

        # plot all debris objects, visible in one color, invisible in another
        for d in debris_list:
            ax.scatter(d.position_xyz[0],
                       d.position_xyz[1],
                       d.position_xyz[2],
                       c='k', s=5)
        # cover invisible with visible in a different color
        for vdp in self.visible_debris_positions:
            ax.scatter(vdp[0],
                       vdp[1],
                       vdp[2],
                       c='r', s=5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()
