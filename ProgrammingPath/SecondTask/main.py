import math

import networkx as nx
import sympy
import itertools

from matplotlib import pyplot as plt


def getDistance(points):
    return math.sqrt((points[0][0] - points[1][0]) ** 2 + (points[0][1] - points[1][1]) ** 2)


def inspect(barrier, segment):
    a = barrier.intersection(segment)
    if len(a) == 2 or (len(a) == 2 and (barrier.encloses(segment.p1) or barrier.encloses(segment.p2))):
        return True
    return False


dataset = [[int(x) for x in input().split()] for i in range(int(input()))]

startPoint = tuple(dataset[0])
endPoint = tuple(dataset[1])
dataset = dataset[2:]
barrierPoints = [[(i[0], i[1]), (i[2], i[3]), (i[4], i[5])] for i in dataset]
barriers = [sympy.Polygon(*barrierPoint) for barrierPoint in barrierPoints]
points = [startPoint, endPoint] + [point for barrierPoint in barrierPoints for point in barrierPoint]
edgesPoints = []
for i in range(len(points)):
    for j in points[i + 1:]:
        segment = sympy.Segment2D(sympy.Point(points[i]), sympy.Point(j))
        flag = True
        for barrier in barriers:
            if inspect(barrier, segment):
                flag = False
                break
        if flag:
            edgesPoints.append((points[i], j))
graph = nx.Graph()
graph.add_edges_from(edgesPoints)
for e in graph.edges:
    graph.add_edge(*e, weight=getDistance(e))
short = nx.dijkstra_path(graph, startPoint, endPoint)
for i in short:
    print(*i)

