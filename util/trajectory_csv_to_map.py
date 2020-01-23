#!/usr/bin/env python

"""
trajectory_csv_to_map.py
Given a trajectory CSV file (generated from rosbag_to_trajectory_csv.py), synthesize a static map where every point
more than x meters from the path is an obstacle.
"""

from __future__ import print_function

import csv
import argparse
import os

import cv2
import yaml
import numpy as np
import tqdm


# distance from the line between u and v to p
def line_distance(u, v, p):
    l = v - u
    l /= np.sqrt(np.dot(l, l))
    r = p - u
    t = np.dot(r, l)

    if t <= 0:
        return np.sqrt(np.dot(r, r))
    elif t >= np.dot(l, v - u):
        x, y = p - v
        return (x * x + y * y) ** 0.5
    else:
        w = np.array([-l[1], l[0]])
        return abs(np.dot(w, r))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--in-csv", type=str, required=True, help="file path of input rosbag file")
    parser.add_argument("--out-folder", type=str, required=True, help="folder in which to place map yaml and png")
    parser.add_argument("--map-name", type=str, required=True)
    parser.add_argument("--track-width", type=float, required=False, default=5.0,
                        help="width of generated track (default: 5.0 m)")
    parser.add_argument("--resolution", type=float, required=False, default=0.1, help="default: 0.1 m")
    args = parser.parse_args()

    path = []
    with open(args.in_csv) as f:
        reader = csv.reader(f, dialect='excel')
        next(reader)  # skip column labels
        for row in reader:
            x = float(row[1])
            y = float(row[2])
            path.append(np.array([x, y]))

    world_min_x = min(x for x, y in path) - args.track_width / 2 - 1.0
    world_min_y = min(y for x, y in path) - args.track_width / 2 - 1.0
    world_max_x = max(x for x, y in path) + args.track_width / 2 + 1.0
    world_max_y = max(y for x, y in path) + args.track_width / 2 + 1.0

    grid_size_x = int((world_max_x - world_min_x) / args.resolution)
    grid_size_y = int((world_max_y - world_min_y) / args.resolution)
    grid = np.zeros((grid_size_y, grid_size_x), dtype=np.uint8)

    def grid_to_world(r, c):
        y = world_min_y + args.resolution * (grid_size_y - r - 1)
        x = world_min_x + args.resolution * c
        return np.array([x, y])

    def world_to_grid(x, y):
        r = int((y - world_min_y) / args.resolution)
        c = int((x - world_min_x) / args.resolution)
        return grid_size_y - r - 1, c

    def segment_dfs(u, v):
        dfs = [world_to_grid(*u)]
        visited = np.zeros(grid.shape, dtype=np.bool)
        visited[dfs[0][0], dfs[0][1]] = True
        grid[dfs[0][0], dfs[0][1]] = 255
        while len(dfs) > 0:
            r, c = dfs.pop()
            for r2, c2 in [(r, c + 1), (r, c - 1), (r + 1, c), (r - 1, c)]:
                if 0 < r2 < grid_size_y and 0 < c2 < grid_size_x and not visited[r2, c2] \
                        and line_distance(u, v, grid_to_world(r2, c2)) < args.track_width / 2:
                    grid[r2, c2] = 255
                    visited[r2, c2] = True
                    dfs.append((r2, c2))

    for u, v in tqdm.tqdm(zip(path[:-1], path[1:])):
        segment_dfs(u, v)

    map_info = {
        "image": "{}.png".format(args.map_name),
        "resolution": args.resolution,
        "origin": [float(world_min_x), float(world_min_y), 0.0],
        "occupied_thresh": 0.9,
        "free_thresh": 0.1,
        "negate": 0
    }

    if not os.path.exists(args.out_folder):
        os.makedirs(args.out_folder)

    with open(os.path.join(args.out_folder, "{}.yaml".format(args.map_name)), "w") as yaml_file:
        yaml.dump(map_info, yaml_file)

    cv2.imwrite(os.path.join(args.out_folder, "{}.png".format(args.map_name)), grid)

    cv2.imshow("grid", grid)
    cv2.waitKey(0)
