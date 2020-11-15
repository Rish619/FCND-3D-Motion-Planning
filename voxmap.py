import numpy as np


def create_voxmap(data, safety_distance=5, max_altitude=None, voxel_size=5):
    """
    Returns a map representation of a 3D configuration space
    based on given obstacle data.
    The `voxel_size` argument sets the resolution of the voxel map.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # maximum altitude
    if max_altitude is None:
        alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))
    else:
        alt_max = max_altitude

    # given the minimum and maximum coordinates we can
    # calculate the size of the map.
    north_size = int(np.ceil((north_max - north_min))) // voxel_size
    east_size = int(np.ceil((east_max - east_min))) // voxel_size
    alt_size = int(alt_max) // voxel_size

    # Create an empty map
    voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)

    for i in range(data.shape[0]):
        # TODO: fill in the voxels that are part of an obstacle with `True`
        #
        # i.e. grid[0:5, 20:26, 2:7] = True
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        bottom = north - d_north - safety_distance - north_min
        top = north + d_north + safety_distance - north_min
        left = east - d_east - safety_distance - east_min
        right = east + d_east + safety_distance - east_min
        bottom = np.floor(bottom / voxel_size)
        top = np.ceil(top / voxel_size)
        left = np.floor(left / voxel_size)
        right = np.ceil(right / voxel_size)
        obstacle = [
            int(np.clip(bottom, 0, north_size - 1)),
            int(np.clip(top, 0, north_size - 1)),
            int(np.clip(left, 0, east_size - 1)),
            int(np.clip(right, 0, east_size - 1)),
        ]
        height = np.ceil((alt + d_alt + safety_distance) / voxel_size)
        height = int(np.clip(height, 0, alt_max - 1))
        voxmap[obstacle[0]:obstacle[1],
               obstacle[2]:obstacle[3],
               0:height] = True

    return voxmap, int(north_min), int(east_min)


def create_voxgrid(data, drone_altitude, safety_distance, voxel_size=5):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.
    The `voxel_size` argument sets the resolution of the voxel map.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min))) // voxel_size
    east_size = int(np.ceil((east_max - east_min))) // voxel_size

    # Create an empty grid
    voxgrid = np.zeros((north_size, east_size))

    for i in range(data.shape[0]):
        # TODO: fill in the voxels that are part of an obstacle with `True`
        #
        # i.e. grid[0:5, 20:26, 2:7] = True
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(north - d_north - north_min -
                    safety_distance) // voxel_size,
                int(north + d_north - north_min +
                    safety_distance) // voxel_size,
                int(east - d_east - east_min - safety_distance) // voxel_size,
                int(east + d_east - east_min + safety_distance) // voxel_size,
            ]

            voxgrid[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3]] = 1

    return voxgrid, int(north_min), int(east_min)