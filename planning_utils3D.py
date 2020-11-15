from enum import Enum
from queue import PriorityQueue
import numpy as np
import time


def create_grid25(data, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        bottom = north - d_north - safety_distance - north_min
        top = north + d_north + safety_distance - north_min
        left = east - d_east - safety_distance - east_min
        right = east + d_east + safety_distance - east_min
        obstacle = [
            int(np.clip(np.floor(bottom), 0, north_size - 1)),
            int(np.clip(np.ceil(top), 0, north_size - 1)),
            int(np.clip(np.floor(left), 0, east_size - 1)),
            int(np.clip(np.ceil(right), 0, east_size - 1)),
        ]
        grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = \
            int(np.rint(alt + d_alt))

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action3D(Enum):
    """
    An action is represented by a 3 element tuple.
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 0, 1)
    EAST = (0, 1, 0, 1)
    NORTH = (-1, 0, 0, 1)
    SOUTH = (1, 0, 0, 1)
    UP = (0, 0, 1, 1)
    DOWN = (0, 0, -1, 1)
    NORTH_WEST = (-1, -1, 0, np.sqrt(2))
    NORTH_EAST = (-1, 1, 0, np.sqrt(2))
    SOUTH_WEST = (1, -1, 0, np.sqrt(2))
    SOUTH_EAST = (1, 1, 0, np.sqrt(2))
    NORTH_DOWN = (-1, 0, -1, np.sqrt(2))
    NORTH_UP = (-1, 0, 1, np.sqrt(2))
    SOUTH_DOWN = (1, 0, -1, np.sqrt(2))
    SOUTH_UP = (1, 0, 1, np.sqrt(2))
    WEST_DOWN = (0, -1, -1, np.sqrt(2))
    WEST_UP = (0, -1, 1, np.sqrt(2))
    EAST_DOWN = (0, 1, -1, np.sqrt(2))
    EAST_UP = (0, 1, 1, np.sqrt(2))
    NORTH_WEST_DOWN = (-1, -1, -1, np.sqrt(3))
    NORTH_WEST_UP = (-1, -1, 1, np.sqrt(3))
    NORTH_EAST_DOWN = (-1, 1, -1, np.sqrt(3))
    NORTH_EAST_UP = (-1, 1, 1, np.sqrt(3))
    SOUTH_WEST_DOWN = (1, -1, -1, np.sqrt(3))
    SOUTH_WEST_UP = (1, -1, 1, np.sqrt(3))
    SOUTH_EAST_DOWN = (1, 1, -1, np.sqrt(3))
    SOUTH_EAST_UP = (1, 1, 1, np.sqrt(3))

    @property
    def cost(self):
        return self.value[3]

    @property
    def delta(self):
        return (self.value[0], self.value[1], self.value[2])


def valid_actions3D(voxmap, current_node, current_action=None):
    """
    Returns a list of valid actions given a grid and current node.
    """
    all_actions = list(Action3D)
    valid_actions_nodes = []
    n, m, t = voxmap.shape[0] - 1, voxmap.shape[1] - 1, voxmap.shape[2] - 1
    x, y, z = current_node

    # To prevent zigzags add a cost to changing action
    # Move previous action first
    if (current_action is not None and
            current_action in all_actions):
        all_actions.remove(current_action)
        all_actions = [current_action] + all_actions

    for new_action in all_actions:
        new_x = x + new_action.delta[0]
        new_y = y + new_action.delta[1]
        new_z = z + new_action.delta[2]
        if (new_x < 0 or new_x > n or
            new_y < 0 or new_y > m or
            new_z < 0 or new_z > t or
                voxmap[new_x, new_y, new_z]):
            pass
        else:
            valid_actions_nodes.append((new_action, (new_x, new_y, new_z)))

    return valid_actions_nodes


def a_star3D2(voxmap, h, start, goal, info=True):
    queuef = PriorityQueue()
    queueb = PriorityQueue()
    queuef.put((0, start))
    queueb.put((0, goal))
    visitedf = set(start)
    visitedb = set(goal)
    midpoint = None

    branchf = {}
    branchb = {}
    found = False

    # To give information about the planning process
    depth = 0
    depth_act = 0
    report_int = 1024

    t0 = time.time()
    isfwd = True
    while ((not found) and (
            not queuef.empty() or not queueb.empty())):
        if isfwd:
            queue = queuef
            branch = branchf
            branch_reverse = branchb
            visited = visitedf
            visited_reverse = visitedb
            target = goal
            begin = start
        else:
            queue = queueb
            branch = branchb
            branch_reverse = branchf
            visited = visitedb
            visited_reverse = visitedf
            target = start
            begin = goal
        isfwd = not isfwd

        item = queue.get()
        current_node = item[1]

        if current_node in visited:
            continue

        visited.add(current_node)
        depth += 1

        if current_node == begin:
            current_cost = 0.0
            current_action = None
        else:
            current_cost = branch[current_node][0]
            current_action = branch[current_node][2]

        if info and depth % report_int == 0:
            print("#Nodes:%s, #Actions:%s, Cost:%.2f, Currenct Node:%s,"
                  " Time:%.2f" % (depth, depth_act, current_cost,
                                  current_node, time.time() - t0))
            report_int *= 2

        if current_node == target:
            found = True
            if info:
                print('Found a path. isfwd:', (not isfwd))
                print("#Nodes:%s, #Actions:%s, Cost:%.2f, Currenct Node:%s,"
                      " Time:%.2f" % (depth, depth_act, current_cost,
                                      current_node, time.time() - t0))
            break
        elif current_node in visited_reverse:
            found = True
            midpoint = current_node
            if info:
                print('found a midpoint')
                print("#Nodes:%s, #Actions:%s, Cost:%.2f, Currenct Node:%s,"
                      " Time:%.2f" % (depth, depth_act, current_cost,
                                      current_node, time.time() - t0))
            break
        else:
            val_act_nod = valid_actions3D(
                voxmap, current_node, current_action)
            for action, next_node in val_act_nod:

                depth_act += 1

                branch_cost = current_cost + action.cost
                if next_node in branch_reverse:
                    h_cost = branch_reverse[next_node][0]
                else:
                    h_cost = h(next_node, target)
                queue_cost = branch_cost + h_cost

                if next_node in branch:
                    cost_in_branch = branch[next_node][0]
                    if branch_cost < cost_in_branch:
                        branch[next_node] = (branch_cost, current_node, action)
                        queue.put((queue_cost, next_node))
                else:
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    path = []
    path_cost = 0
    if found:
        if midpoint is not None:
            if info:
                print("found mid")
            n = midpoint
            path_cost = branchf[n][0]
            while branchf[n][1] != start:
                path.append(branchf[n][1])
                n = branchf[n][1]
            path.append(branchf[n][1])

            path = path[::-1]
            n = midpoint
            path_cost += branchb[n][0]
            while branchb[n][1] != goal:
                path.append(branchb[n][1])
                n = branchb[n][1]
            path.append(branchb[n][1])
        else:
            if info:
                print("found goal")

            # retrace steps
            n = goal
            path_cost = branch[n][0]
            while branch[n][1] != start:
                path.append(branch[n][1])
                n = branch[n][1]
            path.append(branch[n][1])
            path = path[::-1]
    else:
        if info:
            print('**********************')
            print('Failed to find a path!')
            print('**********************')

    return path, path_cost


def heuristic3D(position, goal_position):
    diff = np.array(position) - np.array(goal_position)
    diff_abs = np.abs(diff)

    # sorted diff
    ds = np.sort(diff_abs)
    cost = ds[0] * np.sqrt(3)
    cost += (ds[1] - ds[0]) * np.sqrt(2)
    cost += (ds[2] - ds[1])
    return cost


def heuristic_man(position, goal_position):
    diff = np.array(position) - np.array(goal_position)
    diff_abs = np.abs(diff)

    return np.sum(diff_abs)


def heuristic(position, goal_position):
    if len(position) != 3 or len(goal_position) != 3:
        print("LEN NOT 3: ", position, goal_position)
    return np.linalg.norm(np.array(position) - np.array(goal_position))