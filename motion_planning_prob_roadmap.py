import sys
import pkg_resources
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


# Probabilistic Roadmap

class GridProbabilistic():
	def __init__(self, TARGET_ALTITUDE, SAFETY_DISTANCE):
		t_0 = time.time()
		self.TARGET_ALTITUDE = TARGET_ALTITUDE
		self.SAFETY_DISTANCE = SAFETY_DISTANCE
		
		self.gn_tree = None
		self.graph = None
		self.nodes = []
		
		self.n_steps = 1
		self.nk = 10
		self.n_poly = 2
		self.n_samples = 3000
		self.node_step = int(self.n_samples / self.n_steps)
		
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
		self.north_offset = north_offset
		self.east_offset = east_offset        

		self.polygons = self.extract_polygons(data)
		print("# Polygons:", len(self.polygons))
		
		self.centers = np.array([(p[0].centroid.x, p[0].centroid.y)for p in self.polygons])
		self.c_tree = KDTree(self.centers, metric="euclidean")

		self.xmin = 0.0
		self.xmax = float(grid.shape[0])
		self.ymin = 0.0
		self.ymax = float(grid.shape[1])
		
		self.zmin = 5.0
		# Limit the z axis for the visualization
		self.zmax = 5.0

		print("X | min = {0}, max = {1}".format(self.xmin, self.xmax))
		print("Y | min = {0}, max = {1}".format(self.ymin, self.ymax))
		print("Z | min = {0}, max = {1}".format(self.zmin, self.zmax))        

		t0 = time.time()      
		for step in range(self.n_steps):
			new_nodes = self.sample_nodes()
			graph = self.create_graph(new_nodes)
			te = time.time()
			print("%s.step, #edges:%s, #nodes:%s, #snodes:%s, %.4f secs" %(step, len(self.graph.edges), len(self.graph.nodes), len(self.nodes), te-t0))
			# plotgraphprob(grid, self.graph)
		
		print("Number of edges", len(self.graph.edges))
		te = time.time()
		print('graph took {0} seconds to build, pre took {1}, all took {2}'.format(te-t0, t0 - t_0, te - t_0))


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
			bottom = np.floor(north - d_north - self.SAFETY_DISTANCE - self.north_offset)
			top = np.ceil(north + d_north + self.SAFETY_DISTANCE - self.north_offset)
			left = np.floor(east - d_east - self.SAFETY_DISTANCE - self.east_offset)
			right = np.ceil(east + d_east + self.SAFETY_DISTANCE - self.east_offset)
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
		
	def sample_nodes(self):
		# TODO: sample points randomly
		xvals = np.random.uniform(self.xmin, self.xmax, self.node_step)
		yvals = np.random.uniform(self.ymin, self.ymax, self.node_step)
		zvals = np.random.uniform(self.zmin, self.zmax, self.node_step)

		samples = list(zip(xvals, yvals, zvals))        
		nodes =[]
		n_collide_first = 0
		n_collide_other = 0
		n_collide_all = 0
		n_no_collide = 0
		
		for s in samples:
			#po = Point(s[0], s[1])
			sx = int(s[0])
			sy = int(s[1])
			po = Point(sx, sy)
			
			npo = np.array(Point(s[0], s[1])).reshape(1, -1)
			
			near_polies = self.c_tree.query(npo, k=self.n_poly, return_distance=False)[0]
			is_collision = False
			nidx = 0 
			for near_poly_idx in near_polies:
				nearp, nearph = self.polygons[int(near_poly_idx)]
				if nearp.contains(po):
					if nearph > s[2]:
						is_collision = True
						n_collide_all += 1
						if nidx > 0:
							n_collide_other += 1
							for npidx in near_polies:
								if npidx != near_poly_idx:
									nearp2, nearph2 = self.polygons[int(npidx)]
						else:
							n_collide_first += 1
						break
				nidx += 1
			if not is_collision:
				n_no_collide += 1
				#nodes.append(s)
				sz = int(s[2])
				nodes.append((sx, sy, sz))
		print("# samples, # nodes, # first, # other, # all, # no :", len(samples), len(nodes), n_collide_first, n_collide_other, n_collide_all, n_no_collide)
		# self.nodes = nodes
		return nodes
		
	def can_connect(self, p1, p2):
		line = LineString([p1, p2])
		for p in self.polygons:
			if line.crosses(p[0]) and p[1] >= min(p1[2], p2[2]):
				return False
		else:
			return True
	
	
	def create_graph(self, new_nodes):
		self.nodes = self.nodes + new_nodes
		tg0 = time.time()
		if self.graph is None:
			G = nx.Graph()
		else:
			G = self.graph
		n_tree = KDTree(np.array(self.nodes), metric="euclidean")
		print("tree build in %s sec", time.time() - tg0)
		self.n_tree = n_tree
		l = len(new_nodes)
		i = 0
		k = 2
		for p in new_nodes:
			neighbors = n_tree.query([p], k=self.nk, return_distance=True)

			for idx in range(1,neighbors[0].shape[1]):
				np_idx = neighbors[1][0][idx]
				neig_dist = neighbors[0][0][idx]
				np1 = self.nodes[np_idx]
				if p == np1:
					continue                    
				has_edge = G.has_edge(p,np1)
				if has_edge:
					continue
				if self.can_connect(p, np1):
					G.add_edge(p, np1, weight=neig_dist)
			i += 1
			if (i % k == 0):
				print(i, "of", l, "gr:", len(G.nodes), " t:", time.time() - tg0)
				k = k * 2
				if k > l:
					k == l
		self.graph = G






class MotionPlanning(Drone):

	def __init__(self, connection, GRD, waypoints=[], landing_altitude=0, global_goal=None, local_goal=None, grid_goal=None):
		super().__init__(connection)

		self.target_position = np.array([0.0, 0.0, 0.0])
		self.waypoints = []
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
			self.GRD = GridProbabilistic(TARGET_ALTITUDE, SAFETY_DISTANCE)
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
			# grid_goal = (616, 556)

			if grid_goal[0] >= grid_shape[0]:
				grid_goal = (grid_shape[0] - 1, grid_goal[1])
			elif grid_goal[0] < 0:
				grid_goal = (0, grid_goal[1])            
			if grid_goal[1] >= grid_shape[1]:
				grid_goal = (grid_goal[0], grid_shape[1] - 1)
			elif grid_goal[1] < 0:
				grid_goal = (grid_goal[0], 0)            
						
			goal_obs = grid[grid_goal[0], grid_goal[1]]
			print("gg: ",grid_goal, goal_obs)
			if goal_obs:
				goal_list.append(grid_goal)
				
		if len(grid_start) == 2:
			grid_start = (grid_start[0], grid_start[1] , TARGET_ALTITUDE)
			grid_goal = (grid_goal[0], grid_goal[1] , TARGET_ALTITUDE)
			
		self.GRD.create_graph([grid_start, grid_goal])
		
		
		graph = self.GRD.graph
		nodes = self.GRD.nodes
	
		#plotgraphprob(grid, graph, grid_start, grid_goal, goal_list=goal_list)
		print('Grid Start and Goal: ', grid_start, grid_goal)    
		
		path, cost = a_star_graph(graph, heuristic, grid_start, grid_goal)
		print("Path length:", len(path)," cost:", cost)
		# plotgraphprob(grid, graph, grid_start, grid_goal, goal_list=goal_list, path=path)
		
		# TODO: prune path to minimize number of waypoints
		# TODO (if you're feeling ambitious): Try a different approach altogether!
		pruned_path = prune_path(path)
		print("Pruned Path length: ", len(pruned_path))
		#plotgraphprob(grid, graph, grid_start, grid_goal, goal_list=goal_list, path=pruned_path)

		# print("A* path:")
		# for p in path:
		#     print(p)
			
		# print("Pruned_path:")
		# for p in pruned_path:
		#     print(p)
		head = []
			
		for t in range(len(pruned_path)):
				
			if t == 0:
				head.append(0)
			else:
				head.append(np.arctan2((pruned_path[t][1]-pruned_path[t-1][1]), (pruned_path[t][0]-pruned_path[t-1][0])))

		# Convert path to waypoints
		waypoints = [[int(np.rint(p[0] + north_offset)), int(np.rint(p[1] + east_offset)), TARGET_ALTITUDE, head[i]] for i,p in enumerate(pruned_path)]


		# print("waypoints")
		# print(waypoints)
		# for w in waypoints:
		#    print(w)

		# Set self.waypoints
		self.waypoints = waypoints
		self.GRD.waypoints = waypoints
		# TODO: send waypoints to sim (this is just for visualization of waypoints)
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



def plotgraphprob(grid, graph, start_ne=None, goal_ne=None, goal_list=None, path=None):
	plt.figure(figsize=(24, 12))
	plt.imshow(grid, origin='lower', cmap='Greys') 

	for n1 in graph.nodes:
		plt.scatter(n1[1], n1[0], c='yellow', alpha=0.4, linewidths=5)
			
	# draw edges
	for (n1, n2) in graph.edges:
		plt.plot([n1[1], n2[1]], [n1[0], n2[0]], 'black',linestyle='dashed' , alpha=0.5)
	
	if start_ne is not None:
		plt.plot(start_ne[1], start_ne[0], 'go', markersize=10, markeredgewidth=3, fillstyle='none')
	if goal_ne is not None:
		plt.plot(goal_ne[1], goal_ne[0], 'ro', markersize=10, markeredgewidth=3, fillstyle='none')

	if goal_list is not None:
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


# Graph specific

def convert_graph(edges):
	G = nx.Graph()
	for e in edges:
		p1 = e[0]
		p2 = e[1]
		dist = LA.norm(np.array(p2) - np.array(p1))
		G.add_edge(p1,p2,weight=dist)
	return G

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











if __name__ == "__main__":

	print("main")
	#parser = argparse.ArgumentParser()
	#parser.add_argument('--port', type=int, default=5760, help='Port number')
	#parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
	#args = parser.parse_args()
	
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
			
		
		
	try_count = 10
	try_i = 0
	is_timeout = True
	GRD = None
	waypoints = None
	landing_altitude = 0

	nodes = None
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
		
		
		

