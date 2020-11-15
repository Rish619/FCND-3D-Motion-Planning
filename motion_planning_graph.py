import sys
import pkg_resources
import networkx as nx

import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import numpy.linalg as LA

from planning_utils import a_star, heuristic, create_grid
from planning_utils_graph import create_grid_and_edges, a_star_graph
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


import matplotlib.pyplot as plt
GRD = None

class States(Enum):
	MANUAL = auto()
	ARMING = auto()
	TAKEOFF = auto()
	WAYPOINT = auto()
	LANDING = auto()
	DISARMING = auto()
	PLANNING = auto()

class MotionPlanning(Drone):

	def __init__(self, connection, GRD, waypoints=[]):
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
				if self.takeoff_counter == 1000:
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
			self.GRD = Grid(TARGET_ALTITUDE, SAFETY_DISTANCE)
		else:
			print("grid was already created")
		grid = self.GRD.grid
		edges = self.GRD.edges
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

			if grid_goal[0] >= grid_shape[0]:
				grid_goal = (grid_shape[0] - 1, grid_goal[1])
			elif grid_goal[0] < 0:
				grid_goal = (0, grid_goal[1])            
			if grid_goal[1] >= grid_shape[1]:
				grid_goal = (grid_goal[0], grid_shape[1] - 1)
			elif grid_goal[1] < 0:
				grid_goal = (grid_goal[0], 0)            
						
			goal_obs = grid[grid_goal[0], grid_goal[1]]
			if goal_obs:
				goal_list.append(grid_goal)
		
		graph = convert_graph(edges)
		start_ne_g = closest_point(graph, grid_start)
		goal_ne_g = closest_point(graph, grid_goal)
		plotgraph(grid, edges, grid_start, grid_goal, start_ne_g, goal_ne_g, goal_list=goal_list)
		
		print('Grid Start and Goal: ', grid_start, grid_goal)
		print("Start Nearest and Goal Nearest", start_ne_g, goal_ne_g)
		
		# Run A* to find a path from start to goal
		# TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
		# or move to a different search space such as a graph (not done here)    
		path, cost = a_star_graph(graph, heuristic, start_ne_g, goal_ne_g)
		print("Path length:", len(path)," cost:", cost)
		# plotgraph(grid, edges, grid_start, grid_goal, start_ne_g, goal_ne_g, goal_list=goal_list, path=path)
		
		# TODO: prune path to minimize number of waypoints
		# TODO (if you're feeling ambitious): Try a different approach altogether!    
		pruned_path = prune_path(path)
		print("Pruned Path length: ", len(pruned_path))
		plotgraph(grid, edges, grid_start, grid_goal, start_ne_g, goal_ne_g, goal_list=goal_list, path=pruned_path)
			
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
		waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, head[i]] for i,p in enumerate(pruned_path)]
		# Set self.waypoints
		self.waypoints = waypoints
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

def main(GRD):
	print("main")
	#parser = argparse.ArgumentParser()
	#parser.add_argument('--port', type=int, default=5760, help='Port number')
	#parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
	#args = parser.parse_args()
	
	try_count = 10
	try_i = 0
	is_timeout = True
	GRD = None
	waypoints = None
	nodes = None
	t0 = time.time()

	while (is_timeout and try_i < try_count):
		if GRD is None:
			print("GRD is None")
		else:
			print("GRD is not None")
		conn = MavlinkConnection('tcp:{0}:{1}'.format('127.0.0.1', 5760), timeout=100)
		drone = MotionPlanning(conn, GRD, waypoints=waypoints)
		time.sleep(1)
		drone.start()
		try_i += 1
		if drone.GRD is not None:
			print("GRD = drone.GRD, not none")
			GRD = drone.GRD
		else:
			print("drone.GRD is none")
			return
		print("%s. is_timeout:%s, state:%s, connected:%s, in_mission:%s,"
			  " armed:%s, t:%s" % (
				try_i, drone.is_timeout, drone.flight_state, drone.connected,
				drone.in_mission, drone.armed, time.time() - t0))
		is_timeout = drone.is_timeout
		waypoints = drone.waypoints

def plotgraph(grid, edges, start_ne, goal_ne, start_ne_g, goal_ne_g, goal_list=None, path=None):
	plt.figure(figsize=(24, 12))
	plt.imshow(grid, origin='lower', cmap='Greys') 

	for e in edges:
		p1 = e[0]
		p2 = e[1]
		plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

	plt.plot(start_ne[1], start_ne[0], 'go', markersize=3, markeredgewidth=5, markerfacecolor='none')
	plt.plot(start_ne_g[1], start_ne_g[0], 'gx', markersize=5, markeredgewidth=3)
	plt.plot(goal_ne[1], goal_ne[0], 'ro', markeredgewidth=5, markerfacecolor='none')
	plt.plot(goal_ne_g[1], goal_ne_g[0], 'rx', markersize=5)
	
	#plt.plot(goal_ne[1], goal_ne[0], 'ro', markersize=3, markeredgewidth=5)
	#plt.plot(goal_ne_g[1], goal_ne_g[0], 'rx', markersize=5, markeredgewidth=3)
	
	for g in goal_list:
		plt.plot(g[1], g[0], 'bo', markeredgewidth=2)

	if path is not None:
		pp = np.array(path)
		#pp = np.array(parr)
		plt.plot(pp[:, 1], pp[:, 0], 'g')
		plt.scatter(pp[:,1], pp[:,0])

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

# Graph specific
def closest_point(graph, current_point):
	"""
	Compute the closest point in the `graph`
	to the `current_point`.
	"""
	closest_point = None
	dist = 100000
	for p in graph.nodes:
		d = LA.norm(np.array(p) - np.array(current_point))
		if d < dist:
			closest_point = p
			dist = d
	return closest_point

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

class Grid():
	def __init__(self, TARGET_ALTITUDE, SAFETY_DISTANCE):
		# Read in obstacle map
		data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
		grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
		print("grid# = {0}, edge# = {1}".format(len(grid), len(edges)))
		print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
		# Define starting point on the grid (this is just grid center)
		with open('colliders.csv') as f:
			first_line = f.readline().strip()
		latlon = first_line.split(',')
		self.lon0 = float(latlon[0].strip().split(' ')[1])
		self.lat0 = float(latlon[1].strip().split(' ')[1])      

		self.grid = grid
		self.edges = edges
		self.north_offset = north_offset
		self.east_offset = east_offset



main(GRD)