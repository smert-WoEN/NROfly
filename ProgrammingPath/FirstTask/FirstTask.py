class Graff:

    def __init__(self, points, graph):
        self.points = points
        self.graph = graph

    def findRoad(self, startPos, endPos):
        unvisited = {n: float("inf") for n in self.points}
        unvisited[startPos] = 0
        visited = {}
        parents = {}
        while unvisited:
            minDist = min(unvisited, key=unvisited.get)
            for neighbour, i in self.graph.get(minDist, {}).items():
                if neighbour in visited:
                    continue
                newDist = unvisited[minDist] + self.graph[minDist].get(neighbour, float('inf'))
                if newDist < unvisited[neighbour]:
                    unvisited[neighbour] = newDist
                    parents[neighbour] = minDist
            visited[minDist] = unvisited[minDist]
            unvisited.pop(minDist)
            if minDist == endPos:
                return parents, visited


points1 = ('1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '16', '17')
graph1 = {
    '1': {'2': 8.6, '3': 8.7},
    '2': {'1': 8.6, '3': 9.2, '4': 9.7, '7': 11.0, '10': 9.4},
    '3': {'1': 8.7, '2': 9.2, '4': 10.5, '6': 7.2, '7': 8.9},
    '4': {'2': 9.7, '3': 10.5, '5': 9.1, '10': 8.9, '11': 10.1},
    '5': {'4': 9.1, '13': 11.9},
    '6': {'3': 7.2, '8': 6.0},
    '7': {'2': 11.0, '3': 8.9, '8': 9.3, '9': 10.1, '10': 7.6},
    '8': {'6': 6.0, '7': 9.3, '9': 9.9},
    '9': {'7': 10.1, '8': 9.9, '11': 9.4, '14': 12.3},
    '10': {'2': 9.4, '4': 8.9, '7': 7.6, '12': 11.3},
    '11': {'4': 10.1, '9': 9.4, '12': 12.8, '14': 10.4},
    '12': {'10': 11.3, '11': 12.8, '13': 5.0, '15': 11.1},
    '13': {'5': 11.9, '12': 5.0, '17': 6.0},
    '14': {'9': 12.3, '11': 10.4, '15': 7.0, '16': 12.0},
    '15': {'12': 11.1, '14': 7.0, '16': 12.0},
    '16': {'14': 12.0, '15': 12.0, '17': 11.9},
    '17': {'13': 6.0, '16': 11.9}
}

a = input().split(',')
graphClass = Graff(points1, graph1)
parent, visited = graphClass.findRoad(a[0], a[1])
print(visited[a[1]])
