"""
Ethan Quinlan  Ag Robotics Project     Spring 25

polygon.py -- Point, Edge, and Polygon classes for an environment
"""

import numpy as np
from shapely.geometry import LineString as ShapelyLineString, Point as ShapelyPoint


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"({self.x}, {self.y})"

    def __eq__(self, other):
        return isinstance(other, Point) and self.x == other.x and self.y == other.y

    def getPoint(self):
        return np.array([self.x, self.y])


class Edge:
    def __init__(self, p, q):
        self.p = p
        self.q = q

    def __repr__(self):
        return f"{self.p} -> {self.q}"

    def intersects(self, other):

        p1 = self.p
        q1 = self.q

        p2 = other.p
        q2 = other.q

        line1 = ShapelyLineString([(p1.x, p1.y), (q1.x, q1.y)])
        line2 = ShapelyLineString([(p2.x, p2.y), (q2.x, q2.y)])

        intersection = line1.intersection(line2)
        if not intersection.is_empty and isinstance(intersection, ShapelyPoint):
            return Point(intersection.x, intersection.y)

        return None

    def on_edge(self, pt):
        line = ShapelyLineString([(self.p.x, self.p.y), (self.q.x, self.q.y)])
        s_pt = ShapelyPoint(pt.x, pt.y)
        return line.distance(s_pt) < 1e-10


class Polygon:
    def __init__(self, pts):
        self.n = len(pts)

        self.vertices = []
        for pt in pts:
            self.vertices.append(Point(pt[0], pt[1]))

        self.edges = []
        for i in range(self.n):
            p = self.vertices[i]
            q = self.vertices[(i + 1) % self.n]
            self.edges.append(Edge(p, q))

    def __repr__(self):
        return "\n".join([str(edge) for edge in self.edges])

    def touches(self, pt):
        for edge in self.edges:
            if edge.on_edge(pt):
                return True
        return False
