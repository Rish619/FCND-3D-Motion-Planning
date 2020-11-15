import networkx as nx
import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import numpy.linalg as LA

from planning_utils import heuristic, create_grid
from planning_utils_graph import create_grid_and_edges, a_star_graph
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point, LineString

import matplotlib.pyplot as plt

plt.rcParams['figure.figsize'] = 12, 12

class States(Enum):
	MANUAL = auto()
	ARMING = auto()
	TAKEOFF = auto()
	WAYPOINT = auto()
	LANDING = auto()
	DISARMING = auto()
	PLANNING = auto()

GRD = None

class MotionPlanning(Drone):

	def __init__(self, connection, GRD, waypoints=[], landing_altitude=0, global_goal=None, local_goal=None, grid_goal=None):
		super().__init__(connection)

		self.target_position = np.array([0.0, 0.0, 0.0])
		self.waypoints = []
		# To start an instance with pre-calculated WPs
		if waypoints is not None and len(waypoints) > 0:
			self.waypoints = waypoints
		self.in_mission = True
		self.check_state = {}
		
		
		self.takeoff_counter = 0
		self.timeout = connection._timeout
		self.is_timeout = False

		# initial state
		self.flight_state = States.MANUAL
		
		self.GRD = GRD
		
		# register all your callbacks here
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
		self.register_callback(MsgID.STATE, self.state_callback)

	def local_position_callback(self):
		if self.flight_state == States.TAKEOFF:
			if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
				self.waypoint_transition()
			else:
				# Try to recover if stuck in TAKEOFF
				self.takeoff_counter += 1
				if self.takeoff_counter == 2000:
					self.landing_transition()
					self.disarming_transition()
					self.manual_transition()
					self.stop()
					print("RESETING")
				if (self.takeoff_counter + 1) % 100 == 0:
					print("reset states, current h:%s, tc:%s" % (
						self.local_position[2], self.takeoff_counter))
					lsw = len(self.waypoints)
					if (lsw > 1):
						self.waypoints = self.waypoints[:lsw - 1]
					self.flight_state = States.MANUAL
		elif self.flight_state == States.WAYPOINT:
			if -1.0 * self.local_position[2] < 0.5 * self.target_position[2]:
				# Try to recover if stuck
				self.takeoff_transition()
				print("still in landing position turn back to takeoff")
			dist_to_wp = np.linalg.norm(self.target_position[0:2] -
										self.local_position[0:2])
			if dist_to_wp < 2.0:
				if len(self.waypoints) > 0:
					self.waypoint_transition()
				elif (dist_to_wp < 1.0):
					if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
						self.landing_transition()
	#     elif self.flight_state == States.WAYPOINT:
	#         if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
	#             if len(self.waypoints) > 0:
	#                 self.waypoint_transition()
	#             else:
	#                 if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
	#                     self.landing_transition()

	def velocity_callback(self):
		if self.flight_state == States.LANDING:
			if self.global_position[2] - self.global_home[2] < 0.1:
				if abs(self.local_position[2]) < 0.01:
					self.disarming_transition()

	def state_callback(self):
		if self.in_mission:
			if self.flight_state == States.MANUAL:
				self.arming_transition()
			elif self.flight_state == States.ARMING:
				if self.armed:
					self.plan_path()
			elif self.flight_state == States.PLANNING:
				self.takeoff_transition()
			elif self.flight_state == States.DISARMING:
				if ~self.armed & ~self.guided:
					self.manual_transition()

	def arming_transition(self):
		self.flight_state = States.ARMING
		print("arming transition")
		self.arm()
		self.take_control()

	def takeoff_transition(self):
		self.flight_state = States.TAKEOFF
		print("takeoff transition")
		self.takeoff(self.target_position[2])

	def waypoint_transition(self):
		self.flight_state = States.WAYPOINT
		print("waypoint transition")
		self.target_position = self.waypoints.pop(0)
		print('target position', self.target_position)
		self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

	def landing_transition(self):
		self.flight_state = States.LANDING
		print("landing transition")
		self.land()

	def disarming_transition(self):
		self.flight_state = States.DISARMING
		print("disarm transition")
		self.disarm()
		self.release_control()

	def manual_transition(self):
		self.flight_state = States.MANUAL
		print("manual transition")
		self.stop()
		self.in_mission = False

	def send_waypoints(self):
		print("Sending waypoints to simulator ...")
		data = msgpack.dumps(self.waypoints)
		self.connection._master.write(data)

	def plan_path(self):
		self.flight_state = States.PLANNING
		t0 = time.time()
		print("Searching for a path ...")
		TARGET_ALTITUDE = 5
		SAFETY_DISTANCE = 5

		self.target_position[2] = TARGET_ALTITUDE

		# If WPs calculated previously, send them directly
		# Work around for timeout
		if self.waypoints is not None and len(self.waypoints) > 0:
			time.sleep(2)
			print("waypoints:")
			print(self.waypoints)
			print(self.flight_state, self.in_mission, self.connected)
			self.send_waypoints()
			return


		print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
																		self.local_position))
		if self.GRD is None:
			print("creating grid")
			self.GRD = GridRRT(TARGET_ALTITUDE, SAFETY_DISTANCE)
		else:
			print("grid was already created")
			
		grid = self.GRD.grid
		north_offset = self.GRD.north_offset
		east_offset = self.GRD.east_offset
		lon0 = self.GRD.lon0
		lat0 = self.GRD.lat0

		# TODO: convert start position to current position rather than map center
		# TODO: set home position to (lon0, lat0, 0)
		self.set_home_position(lat0, lon0, 0)

		# TODO: retrieve current global position
		# TODO: convert to current local position using global_to_local()
		local_pos = global_to_local(self.global_position, global_home=self.global_home)
		north, east, att = local_pos
		grid_start = (int(np.rint(north - north_offset)), int(np.rint(east - east_offset)))
		print("Grid Start: ",grid_start)
		
		# Set goal as some arbitrary position on the grid
		dist_idx = 100.0
		goal_obs = True
		goal_try = 0
		goal_list = []
		grid_shape = grid.shape
		while goal_obs and goal_try < 100:
			goal_try += 1
			change = np.random.rand(3)
			change -= 0.5
			print("change", change)
			goal = (self.global_home[0] + change[0] / dist_idx,
					self.global_home[1] + change[1] / (dist_idx),
					self.global_home[2] + change[2] * 10.0)
			print("Goal Global: ", goal)
			local_goal = global_to_local(goal, global_home=self.global_home)
			print("Goal Local: ", local_goal)
			ng, eg, ag = local_goal

			grid_goal = (int(np.rint(ng - north_offset)), int(np.rint(eg - east_offset)))

			print(grid_goal[1], grid_shape[1] - 1)
			if grid_goal[0] >= grid_shape[0]:
				grid_goal = (grid_shape[0] - 1, grid_goal[1])
			elif grid_goal[0] < 0:
				grid_goal = (0, grid_goal[1])            
			if grid_goal[1] >= grid_shape[1]:
				grid_goal = (grid_goal[0], grid_shape[1] - 1)
			elif grid_goal[1] < 0:
				grid_goal = (grid_goal[0], 0)            
						
			goal_obs = grid[grid_goal[0], grid_goal[1]]
			print("Selected Grid Goal, Obstacle?: ",grid_goal, goal_obs)
			if goal_obs:
				goal_list.append(grid_goal)
		
		print('Grid Start and Goal: ', grid_start, grid_goal) 
		n_vertices = 4000
		dt = 8
		plotrrt(grid, grid_start, grid_goal, goal_list=goal_list, path=None, edges=None)
		
		rrt = self.GRD.generate_RRT(grid_start, grid_goal, n_vertices, dt)
		
		plotrrt(grid, grid_start, grid_goal, goal_list=goal_list, path=None, edges=rrt.edges)

		path, cost = a_star_graph(rrt.tree, heuristic, grid_start, grid_goal)
		print("Path length:", len(path)," cost:", cost)

		plotrrt(grid, grid_start, grid_goal, goal_list=goal_list, path=path, edges=rrt.edges)
		
		# TODO: prune path to minimize number of waypoints
		# TODO (if you're feeling ambitious): Try a different approach altogether!    
		pruned_path = prune_path(path)
		print("Pruned Path length: ", len(pruned_path))
		plotrrt(grid, grid_start, grid_goal, goal_list=goal_list, path=path, edges=rrt.edges)
		
		# print("A* path:")
		# for p in path:
		#     print(p)
			
		# print("Pruned_path:")
		# for p in pruned_path:
		#     print(p)

		# Convert path to waypoints
		head = []
			
		for t in range(len(pruned_path)):
				
			if t == 0:
				head.append(0)
			else:
				head.append(np.arctan2((pruned_path[t][1]-pruned_path[t-1][1]), (pruned_path[t][0]-pruned_path[t-1][0])))

		# Convert path to waypoints
		waypoints = [[int(np.rint(p[0] + north_offset)), int(np.rint(p[1] + east_offset)), TARGET_ALTITUDE, head[i]] for i,p in enumerate(pruned_path)]

		# Set self.waypoints
		self.waypoints = waypoints
		# TODO: send waypoints to sim
		# (this is just for visualization of waypoints)
		t_int = time.time() - t0

		# If timeout, don't send WPs
		# End this instance, main will start a new instance
		if t_int < self.timeout:
				print("no timeout, continue")
				self.send_waypoints()
		else:
				print("timeout, send wp to a new drone instance")
				self.is_timeout = True
				self.disarming_transition()
				self.manual_transition()

	def start(self):
		self.start_log("Logs", "NavLog.txt")

		print("starting connection")
		self.connection.start()

		# Only required if they do threaded
		# while self.in_mission:
		#    pass

		self.stop_log()

	
def plotrrt(grid, start_ne, goal_ne, goal_list=None, path=None, edges=None):
	plt.figure(figsize=(24, 12))
	plt.imshow(grid, origin='lower', cmap='Greys') 

			
	# draw edges
	if edges is not None:
		for (n1, n2) in edges:
			plt.plot([n1[1], n2[1]], [n1[0], n2[0]], 'y-', alpha=0.5)

	if start_ne is not None:
		plt.plot(start_ne[1], start_ne[0], 'go', markersize=10, markeredgewidth=3, fillstyle='none')
	if goal_ne is not None:
		plt.plot(goal_ne[1], goal_ne[0], 'ro', markersize=10, markeredgewidth=3, fillstyle='none')
	
	for g in goal_list:
		plt.plot(g[1], g[0], 'bd', markeredgewidth=2)

	if path is not None:
		if len(path) > 0:
			path_pairs = zip(path[:-1], path[1:])
			for (n1, n2) in path_pairs:
				plt.plot([n1[1], n2[1]], [n1[0], n2[0]], 'green',linewidth=4)
			plt.plot([path[-1][1], goal_ne[1]], [path[-1][0], goal_ne[0]],linewidth=4)    


		
	plt.xlabel('EAST')
	plt.ylabel('NORTH')
	plt.show()

def point(p):
	return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
	m = np.concatenate((p1, p2, p3), 0)
	det = np.linalg.det(m)
	return abs(det) < epsilon

def prune_path(path):
	pruned_path = [p for p in path]
	# TODO: prune the path!
	i = 0
	while i < len(pruned_path) - 2:
		p1 = point(pruned_path[i])
		p2 = point(pruned_path[i+1])
		p3 = point(pruned_path[i+2])
		if collinearity_check(p1,p2,p3):
			pruned_path.remove(pruned_path[i+1])
		else:
			i += 1
	return pruned_path



class RRT:
	def __init__(self, x_init):
		# A tree is a special case of a graph with
		# directed edges and only one path to any vertex.
		self.tree = nx.DiGraph()
		self.tree.add_node(x_init)
				
	def add_vertex(self, x_new):
		self.tree.add_node(tuple(x_init))
	
	def add_edge(self, x_near, x_new, u, w):
		#print("Before tree.add_edge", len(self.vertices))
		self.tree.add_edge(tuple(x_near), tuple(x_new), orientation=u, weight=w)
		#print("After tree.add_edge", len(self.vertices))
		
	@property
	def vertices(self):
		return self.tree.nodes()
	
	@property
	def edges(self):
		return self.tree.edges()


class GridRRT():
	def __init__(self, TARGET_ALTITUDE, SAFETY_DISTANCE):
		
		self.TARGET_ALTITUDE = TARGET_ALTITUDE
		self.SAFETY_DISTANCE = SAFETY_DISTANCE
		
		n_samples = 300        
				
		# Read in obstacle map
		data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
		
		grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
		print("grid# = {0}".format(len(grid)))
		print("grid shape = {0}".format(grid.shape))
		print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
		# Define starting point on the grid (this is just grid center)
		with open('colliders.csv') as f:
			first_line = f.readline().strip()
		latlon = first_line.split(',')
		self.lon0 = float(latlon[0].strip().split(' ')[1])
		self.lat0 = float(latlon[1].strip().split(' ')[1])      

		self.grid = grid
		self.grid_shape = grid.shape
		self.north_offset = north_offset
		self.east_offset = east_offset
		self.polygons = self.extract_polygons(data)
		
	def extract_polygons(self,data):

		polygons = []
		for i in range(data.shape[0]):
			north, east, alt, d_north, d_east, d_alt = data[i, :]

			# TODO: Extract the 4 corners of the obstacle
			# 
			# NOTE: The order of the points matters since
			# `shapely` draws the sequentially from point to point.
			#
			# If the area of the polygon is 0 you've likely got a weird
			# order.
			bottom = north - d_north - self.SAFETY_DISTANCE - self.north_offset
			top = north + d_north + self.SAFETY_DISTANCE - self.north_offset
			left = east - d_east - self.SAFETY_DISTANCE - self.east_offset
			right = east + d_east + self.SAFETY_DISTANCE - self.east_offset            
			bottom_right = (bottom, right)
			top_right = (top, right)
			top_left = (top, left)
			bottom_left = (bottom, left)
			corners = [bottom_right,
					   top_right,
					   top_left,
					   bottom_left]

			# TODO: Compute the height of the polygon
			height = alt + d_alt + self.SAFETY_DISTANCE

			# TODO: Once you've defined corners, define polygons
			p = Polygon(corners)
			polygons.append((p, height))

		return polygons        

	def sample_state(self):
		xval = np.random.uniform(0, self.grid_shape[0])
		yval = np.random.uniform(0, self.grid_shape[1])
		return (xval, yval)
	
	def nearest_neighbor(self, x_rand, rrt):
		 # TODO: complete
		closest_dist = 100000
		closest_vertex = None
		x_rand = np.array(x_rand)

		for v in rrt.vertices:
			d = np.linalg.norm(x_rand - np.array(v[:2]))
			if d < closest_dist:
				closest_dist = d
				closest_vertex = v
		return closest_vertex
	
	def select_input(self, x_rand, x_near):
		 # TODO: complete
		return np.arctan2(x_rand[1] - x_near[1], x_rand[0] - x_near[0])
	
	def new_state(self, x_near, u, dt):
		# TODO: complete
		nx = x_near[0] + np.cos(u)*dt
		ny = x_near[1] + np.sin(u)*dt
		if nx < 1:
			nx = 1
		elif nx > self.grid_shape[0] - 2:
			nx = self.grid_shape[0] - 2
		if ny < 1:
			ny = 1
		elif ny > self.grid_shape[1] - 2:
			ny = self.grid_shape[1] - 2            
		return [nx, ny]
	
	def can_connect(self, p1, p2):
		#line = LineString(p1, p2)
		line = LineString([p1, p2])
		h = self.TARGET_ALTITUDE
		if len(p1) == 3:
			h = min(p1[2], p2[2])
		for p in self.polygons:
			if line.crosses(p[0]) and p[1] >= h:
				return False
		else:
			return True    
	
	def can_reach(self, x_near, goal, u, dt):
		dx = np.float(np.abs(goal[0] - x_near[0]))
		dy = np.float(np.abs(goal[1] - x_near[1]))
		tx = dx / np.cos(u)
		ty = dy / np.sin(u)
		if tx < dt and ty < dt:
			print("can reach from %s to %s; u:%s, dt:%s, tx:%s, ty:%s, dx:%s, dy:%s" %(
			x_near, goal, u, dt, tx, ty, dx, dy))
			return True
		else:
			print("can't reach from %s to %s; u:%s, dt:%s, tx:%s, ty:%s, dx:%s, dy:%s" %(
			x_near, goal, u, dt, tx, ty, dx, dy))
						
			return False
	
	def generate_RRT(self, x_init, goal, num_vertices, dt):
		t0 = time.time()
		rrt = RRT(x_init)
		grid = self.grid
		k = 2
		rdt = dt
		for i in range(num_vertices):
			x_rand = self.sample_state()
			# sample states until a free state is found
			while grid[int(x_rand[0]), int(x_rand[1])] == 1:
				x_rand = self.sample_state()
			x_near = self.nearest_neighbor(x_rand, rrt)
			u = self.select_input(x_rand, x_near)
			x_new = self.new_state(x_near, u, rdt)
			
			if self.can_connect(x_new, x_near):
				# the orientation `u` will be added as metadata to
				# the edge
				dist = LA.norm(np.array(x_new) - np.array(x_near))
				rrt.add_edge(x_near, x_new, u, dist)
			if (i % k == 0):
				print("# Vertices:", len(rrt.vertices), "/", num_vertices,
					  "# Edges:", len(rrt.edges), "t:", time.time() - t0)
				k *= 2
		print("END # Vertices:", len(rrt.vertices), "/", num_vertices,
			  "# Edges:", len(rrt.edges), "t:", time.time() - t0)
		x_near = self.nearest_neighbor(goal, rrt)
		while x_near != goal:
			u = self.select_input(goal, x_near)
			if self.can_reach(x_near, goal, u, rdt):
				x_new = goal
			else:
				x_new = self.new_state(x_near, u,rdt)
			if self.can_connect(x_new, x_near):
				# the orientation `u` will be added as metadata to
				# the edge
				dist = LA.norm(np.array(x_new) - np.array(x_near))
				rrt.add_edge(x_near, x_new, u, dist)
				x_near = x_new
			else:
				print("no path from nearest %s to goal %s" %(x_near, goal))
				break
		
		self.rrt = rrt
		return rrt

	

if __name__ == "__main__":

		print("main")

		parser = argparse.ArgumentParser()
		parser.add_argument('--port', type=int, default=5760, help='Port number')
		parser.add_argument('--host', type=str, default='127.0.0.1',
							help="host address, i.e. '127.0.0.1'")
		parser.add_argument('--global_goal',
							type=str,
							default=None,
							help='Global location of the goal')
		parser.add_argument('--local_goal',
							type=str,
							default=None,
							help='Local location of the goal')
		parser.add_argument('--grid_goal',
							type=str,
							default=None,
							help='Grid location of the goal')
		
		args = parser.parse_args()

		global_goal = None
		local_goal = None
		grid_goal = None
		
		if args.global_goal is not None:
			global_goal = string_to_tuple(args.global_goal)
		if args.local_goal is not None:
			local_goal = string_to_tuple(args.local_goal)
		if args.grid_goal is not None:
			grid_goal = string_to_tuple(args.grid_goal)
			
		
		
		
		try_count = 5
		try_i = 0
		is_timeout = True
		GRD = None
		waypoints = None
		landing_altitude = 0

		t0 = time.time()
		
		while (is_timeout and try_i < try_count):
			if GRD is None:
				print("GRD is None")
			else:
				print("GRD is not None")
			conn = MavlinkConnection('tcp:{0}:{1}'.format('127.0.0.1', 5760), timeout=1000)
			drone = MotionPlanning(conn, GRD, waypoints = waypoints, landing_altitude=landing_altitude, global_goal=global_goal,
                               local_goal=local_goal,
                               grid_goal=grid_goal)
			time.sleep(1)

			drone.start()
			try_i += 1
			if drone.GRD is not None:
				print("GRD = drone.GRD, not none")
				GRD = drone.GRD
			else:
				print("drone.GRD is none")
			print("%s. is_timeout:%s, state:%s, connected:%s, in_mission:%s,"
				" armed:%s, t:%s" % (
					try_i, drone.is_timeout, drone.flight_state, drone.connected,
					drone.in_mission, drone.armed, time.time() - t0))
			is_timeout = drone.is_timeout
			waypoints = drone.waypoints

