import argparse
import os
from typing import List, Tuple

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString
import numpy as np


def get_angle_to_x_axis(point_a: np.ndarray, point_b: np.ndarray) -> float:
    vec = point_b - point_a
    angle_rad = np.arctan2(vec[1], vec[0])
    angle_deg = np.degrees(angle_rad)
    if angle_deg < 0:
        return 360 + angle_deg
    return angle_deg


def sort_points_ccw(points):
    centroid = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - centroid[1], points[:, 0] - centroid[0])
    sorted_indices = np.argsort(angles)
    sorted_points = points[sorted_indices]
    return sorted_points


def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """
    o_index = 0
    r_index = 0
    vertex_o = sort_points_ccw(np.array(original_shape.exterior.coords)[:-1])
    vertex_o = np.append(vertex_o, vertex_o[:2], axis=0)
    vertex_r = np.array([(0, -r), (r, 0), (0, r), (-r, 0), (0, -r), (r, 0)])
    vertex_new = []
    while True:
        new_point = np.add(vertex_o[o_index], vertex_r[r_index])
        vertex_new.append(new_point)
        r_angle = get_angle_to_x_axis(vertex_r[r_index], vertex_r[r_index + 1])
        o_angle = get_angle_to_x_axis(vertex_o[o_index], vertex_o[o_index + 1])
        # print(f"r1, r2: {vertex_r[r_index]}, {vertex_r[r_index + 1]}, r_angle: {r_angle}")
        # print(f"o1, o2: {vertex_o[o_index]}, {vertex_o[o_index + 1]}, o_angle: {o_angle}")
        if o_index > len(original_shape.exterior.coords) - 2:
            o_angle += 360
        if r_index > 3:
            r_angle += 360
        if r_angle < o_angle:
            r_index += 1
        elif r_angle > o_angle:
            o_index += 1
        else:
            r_index += 1
            o_index += 1
        if r_index == 4 and o_index == len(original_shape.exterior.coords) - 1:
            break
    return Polygon(vertex_new)

def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    v_graph_edges = []
    vertices = set(tuple(coord) for obstacle in obstacles for coord in obstacle.boundary.coords)
    if source:
        vertices.add(source)
    if dest:
        vertices.add(dest)
    vertices = list(vertices)
    for vertex in vertices[::-1]:
        vertices.pop()
        for edge in [LineString([vertex, other_vertex]) for other_vertex in vertices]:
            if any(edge.intersects(obs) and not edge.touches(obs) for obs in obstacles):
                continue
            if any(edge.equals(other_edge) for other_edge in v_graph_edges):
                continue
            v_graph_edges.append(edge)
    return v_graph_edges
    

def get_shortest_path(visibility_graph: List[LineString], src, dest):
    """
    literally just dijekstra's
    """
    vertices = set(vertex for edge in visibility_graph for vertex in tuple(edge.coords))
    edges = set((tuple(line.coords[0]), tuple(line.coords[1])) for line in visibility_graph)
    unvisited = list(vertices)  # type: list[list[int]]
    dist = {}  # type: dict[list[int], int]
    prev = {}  # type: dict[list[int], list[int]]
    for vertex in vertices:
        dist[vertex] = np.inf
        prev[vertex] = None
    dist[src] = 0

    while unvisited:
        current_vertex = min(
            {vertex: dist[vertex] for vertex in unvisited}, key=dist.get
        )
        unvisited.remove(current_vertex)
        if current_vertex == dest:
            path = [current_vertex]  # type: list[LineString]
            while prev[current_vertex]:
                path.insert(0, prev[current_vertex])
                current_vertex = prev[current_vertex]
            return path, LineString(path).length

        # list of unvisited vertices such that a line connects them and the current vertex current_vertex
        current_vertex_neighbors = [
            vertex
            for vertex in unvisited
            if (tuple(current_vertex), tuple(vertex)) in edges or (tuple(vertex), tuple(current_vertex)) in edges
        ]
        for v in current_vertex_neighbors:
            alt_dist = dist[current_vertex] + LineString([current_vertex, v]).length
            if alt_dist < dist[v]:
                dist[v] = alt_dist
                prev[v] = current_vertex


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(" ")
    dist = float(dist)
    source = tuple(map(float, source.split(",")))
    return source, dist


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "Robot",
        help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices",
    )
    parser.add_argument(
        "Obstacles", help="A file that contains the obstacles in the map"
    )
    parser.add_argument(
        "Query", help="A file that contains the ending position for the robot."
    )
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, "r") as f:
        for line in f.readlines():
            ob_vertices = line.split(" ")
            if "," not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(","))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    with open(robot, "r") as f:
        source, dist = get_points_and_dist(f.readline())

    # #==========================test==============================================
    # c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    # lines = get_visibility_graph(c_space_obstacles)
    # exit()
    # #========================end test============================================

    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()

    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()

    # step 2:

    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()

    # step 3:
    with open(query, "r") as f:
        dest = tuple(map(float, f.readline().split(",")))

    lines = get_visibility_graph(c_space_obstacles, source, dest)
    shortest_path, cost = get_shortest_path(lines, source, dest)

    plotter3 = Plotter()
    plotter3.add_robot(source, dist)
    plotter3.add_obstacles(workspace_obstacles)
    plotter3.add_robot(dest, dist)
    plotter3.add_visibility_graph(lines)
    plotter3.add_shorterst_path(list(shortest_path))

    plotter3.show_graph()
