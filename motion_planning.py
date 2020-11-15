import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)


def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


def prune_path(path, max_p=50):
    pruned_path = [p for p in path]
    # prune the path!
    i = 0
    ri = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i + 1])
        p3 = point(pruned_path[i + 2])
        if collinearity_check(p1, p2, p3) and ri < max_p:
            pruned_path.remove(pruned_path[i + 1])
            ri += 1
        else:
            i += 1
            ri = 0
    return pruned_path


class MotionPlanning(Drone):

    def __init__(self, connection, global_goal=None, local_goal=None,
                 grid_goal=None, waypoints=[]):
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

        self.global_goal = global_goal
        self.local_goal = local_goal
        self.grid_goal = grid_goal
        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION,
                               self.local_position_callback)
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
            if dist_to_wp < 1.0 * (np.linalg.norm(self.local_velocity[0:2])):
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                elif (dist_to_wp < 1.0):
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.01:
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
        self.cmd_position(self.target_position[0], self.target_position[1],
                          self.target_position[2], self.target_position[3])

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
        SAFETY_DISTANCE = 6

        # Number of steps to take in a single action
        max_move = 5
        # Number of nodes to prune at max
        max_nodes_to_prune = 50
        max_prune = int(max_nodes_to_prune / max_move)

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

        np.set_printoptions(precision=3)
        print('global home {0}, position {1}, local position {2}'.format(
              np.array(self.global_home),
              np.array(self.global_position),
              np.array(self.local_position)))

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64',
                          skiprows=2)
        # Define a grid for a particular altitude and
        # safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE,
                                                      SAFETY_DISTANCE)
        grid_shape = grid.shape
        print("North offset = {0}, east offset = {1}".format(north_offset,
                                                             east_offset))

        # Define starting point on the grid (this is just grid center)
        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline().strip()
        latlon = first_line.split(',')
        lon0 = float(latlon[0].strip().split(' ')[1])
        lat0 = float(latlon[1].strip().split(' ')[1])

        # TODO: convert start position to current position
        # rather than map center
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lat0, lon0, 0)

        # TODO: retrieve current global position
        # TODO: convert to current local position using global_to_local()
        local_pos = global_to_local(self.global_position,
                                    global_home=self.global_home)
        north, east, att = local_pos

        grid_start = (int(np.rint(north - north_offset)),
                      int(np.rint(east - east_offset)))
        print("Grid Start: ", grid_start)

        if self.global_goal is not None:
            if len(self.global_goal) == 2:
                self.global_goal = (self.global_goal[0], self.global_goal[1],
                                    TARGET_ALTITUDE)
            self.global_to_grid(north_offset, east_offset)
        elif self.local_goal is not None:
            self.local_to_grid(north_offset, east_offset)

        goal_list = []

        # If goal location is in an obstacle
        goal_obs = True
        if self.grid_goal is not None:
            self.place_goal_in_grid(grid_shape)
            grid_goal = tuple(map(int, self.grid_goal))

            print("Goal is set to {0} with the parameter".format(grid_goal))
            goal_obs = grid[grid_goal[0], grid_goal[1]]
            if goal_obs:
                goal_list.append(grid_goal)

        # randomly select a goal
        dist_idx = 100.0
        goal_try = 0
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
            grid_goal = (int(np.rint(ng - north_offset)),
                         int(np.rint(eg - east_offset)))

            if grid_goal[0] > grid_shape[0] - 2:
                grid_goal = (grid_shape[0] - 2, grid_goal[1])
            elif grid_goal[0] < 1:
                grid_goal = (1, grid_goal[1])
            if grid_goal[1] > grid_shape[1] - 2:
                grid_goal = (grid_goal[0], grid_shape[1] - 2)
            elif grid_goal[1] < 1:
                grid_goal = (grid_goal[0], 1)

            goal_obs = grid[grid_goal[0], grid_goal[1]]
            if goal_obs:
                goal_list.append(grid_goal)

        print('Grid Start and Goal: ', grid_start, grid_goal)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2)
        # to your A* implementation
        # or move to a different search space such as a graph (not done here)
        path, cost = a_star(grid, heuristic, grid_start, grid_goal, max_move=5)
        print("Path length:", len(path), " cost:", cost)

        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious):
        # Try a different approach altogether!
        pruned_path = prune_path(path, max_prune)
        print("Pruned Path length: ", len(pruned_path))

        # print("A* path:")
        # for p in path:
        #    print(p)

        # print("Pruned_path:")
        # for p in pruned_path:
        #    print(p)
        head = []
        
        for t in range(len(pruned_path)):
            
            if t == 0:
                head.append(0)
            else:
                head.append(np.arctan2((pruned_path[t][1]-pruned_path[t-1][1]), (pruned_path[t][0]-pruned_path[t-1][0])))
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset,
                      p[1] + east_offset,
                      TARGET_ALTITUDE,
                      head[i]] for i,p in enumerate(pruned_path)]

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

    def local_to_grid(self, north_offset, east_offset):
        if len(self.local_goal) == 2:
            ng, eg = self.local_goal
        elif len(self.local_goal) == 3:
            ng, eg, ag = self.local_goal
        grid_goal = grid_goal = (int(np.rint(ng - north_offset)),
                                 int(np.rint(eg - east_offset)))
        self.grid_goal = grid_goal

    def global_to_grid(self, north_offset, east_offset):
        self.local_goal = global_to_local(self.global_goal,
                                          global_home=self.global_home)
        self.local_to_grid(north_offset, east_offset)

    def place_goal_in_grid(self, grid_shape):
        grid_goal = self.grid_goal
        if grid_goal[0] > grid_shape[0] - 2:
            grid_goal = (grid_shape[0] - 2, grid_goal[1])
        elif grid_goal[0] < 1:
            grid_goal = (1, grid_goal[1])
        if grid_goal[1] > grid_shape[1] - 2:
            grid_goal = (grid_goal[0], grid_shape[1] - 2)
        elif grid_goal[1] < 1:
            grid_goal = (grid_goal[0], 1)
        self.grid_goal = grid_goal


def string_to_tuple(location_str):
    loc_list = location_str.replace("(", "").replace(")", "").split(",")
    if len(loc_list) == 2:
        return (float(loc_list[0].strip()),
                float(loc_list[1].strip()))
    elif len(loc_list) == 3:
        return (float(loc_list[0].strip()),
                float(loc_list[1].strip()),
                float(loc_list[2].strip()))


if __name__ == "__main__":
    # main(grid_goal = (488, 182))

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

    timeout = 60
    try_count = 3
    try_i = 0
    is_timeout = True
    waypoints = None
    t0 = time.time()

    # If there is a timeout with a drone instance,
    # the instance terminates itself
    # main will transfer the WPs to a new instance
    while (is_timeout and try_i < try_count):
        conn = MavlinkConnection('tcp:{0}:{1}'.format('127.0.0.1', 5760),
                                 timeout=timeout)
        drone = MotionPlanning(conn,
                               global_goal=global_goal,
                               local_goal=local_goal,
                               grid_goal=grid_goal,
                               waypoints=waypoints)
        time.sleep(1)
        drone.start()
        try_i += 1
        print("%s. is_timeout:%s, state:%s, connected:%s, in_mission:%s,"
              " armed:%s, t:%s" % (
                  try_i, drone.is_timeout, drone.flight_state, drone.connected,
                  drone.in_mission, drone.armed, time.time() - t0))
        is_timeout = drone.is_timeout
        waypoints = drone.waypoints
