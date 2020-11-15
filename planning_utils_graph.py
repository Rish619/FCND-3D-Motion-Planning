from queue import PriorityQueue
import numpy as np
from scipy.spatial import Voronoi
from bresenham import bresenham
import time


def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
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
    # Initialize an empty list for Voronoi points
    points = []
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

            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)

    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]

        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if ((np.amin(c) < 0) or
                (c[0] >= grid.shape[0]) or
                    (c[1] >= grid.shape[1])):
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, edges, int(north_min), int(east_min)


def a_star_graph(graph, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    depth = 0
    depth_act = 0
    report_int = 2

    t0 = time.time()
    while not queue.empty():
        depth += 1

        item = queue.get()
        current_node = item[1]

        if current_node in visited:
            continue

        visited.add(current_node)

        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]
        dgoal = h(current_node, goal)
        if depth % report_int == 0:
            report_int *= 2
            print("#Nodes:%s, #Actions:%s, Cost:%.2f, HCost:%.2f,"
                  " Currenct Node:%s, Time:%.2f" %
                  (depth, depth_act, current_cost, dgoal, current_node,
                   time.time() - t0))

        if current_node == goal:
            print('Found a path.')
            found = True
            print("#Nodes:%s, #Actions:%s, Cost:%.2f, HCost:%.2f,"
                  " Currenct Node:%s, Time:%.2f" %
                  (depth, depth_act, current_cost, dgoal, current_node,
                   time.time() - t0))
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + h(next_node, goal)

                if next_node in branch:
                    cost_in_branch = branch[next_node][0]
                    if new_cost < cost_in_branch:
                        branch[next_node] = (new_cost, current_node)
                        queue.put((new_cost, next_node))
                else:
                    branch[next_node] = (new_cost, current_node)
                    queue.put((new_cost, next_node))

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
        # while branch[n][1] != start:
        #    path.append(branch[n][2])
        #    n = branch[n][1]
        # path.append(branch[n][2])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    return path[::-1], path_cost
