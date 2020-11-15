from enum import Enum
from queue import PriorityQueue
import numpy as np
import time


def create_grid(data, drone_altitude, safety_distance):
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
        if alt + d_alt + safety_distance > drone_altitude:
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
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1
    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node, current_action=None, move=1):
    """
    Returns a list of valid actions given a grid and current node.
    """
    all_actions = list(Action)
    valid_actions_nodes = []
    n, m = grid.shape[0] - 1, grid.shape[1] - 1

    # To prevent zigzags add a cost to changing action
    # Move previous action first
    if (current_action is not None and
            current_action in all_actions):
        all_actions.remove(current_action)
        all_actions = [current_action] + all_actions

    for new_action in all_actions:
        new_x = current_node[0] + new_action.delta[0] * move
        new_y = current_node[1] + new_action.delta[1] * move

        if (new_x < 0 or new_x > n or
            new_y < 0 or new_y > m or
                grid[new_x, new_y]):
            pass
        else:
            valid_actions_nodes.append((new_action, (new_x, new_y)))

    return valid_actions_nodes


def a_star(grid, h, start, goal, max_move=1):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    # To give information about the planning process
    depth = 0
    depth_act = 0
    report_int = 1024

    t0 = time.time()
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        current_q_cost = item[0]

        move = max_move

        if current_node in visited:
            continue

        visited.add(current_node)
        depth += 1

        if current_node == start:
            current_cost = 0.0
            current_action = None
        else:
            current_cost = branch[current_node][0]
            current_action = branch[current_node][2]
        if depth % report_int == 0:
            print("#Nodes:%s, #Actions:%s, Cost:%.2f, Currenct Node:%s,"
                  " Time:%.2f" % (depth, depth_act, current_cost,
                                  current_node, time.time() - t0))
            report_int *= 2

        current_h_cost = current_q_cost - current_cost

        if current_h_cost < np.sqrt(2) * float(max_move):
            move = 1
        else:
            move = max_move

        if current_node == goal:
            print('Found a path.')
            found = True
            print("#Nodes:%s, #Actions:%s, Cost:%.2f, Currenct Node:%s,"
                  " Time:%.2f" % (depth, depth_act, current_cost,
                                  current_node, time.time() - t0))
            break
        else:
            val_act_nod = valid_actions(
                grid, current_node, current_action, move)
            for action, next_node in val_act_nod:

                depth_act += 1

                action_cost = action.cost * move
                branch_cost = current_cost + action_cost
                h_cost = h(next_node, goal)
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

        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    return path[::-1], path_cost


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))