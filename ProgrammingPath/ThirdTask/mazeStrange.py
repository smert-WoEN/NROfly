import requests
import numpy as np
import cv2
import networkx as nx

url = "https://stepik.org/media/attachments/course/128568/maze0.png"
resp = requests.get(url, stream=True, files=True).raw
image = np.array(bytearray(resp.read()), dtype=np.uint8)
image = cv2.imdecode(image, cv2.IMREAD_COLOR)
check = np.array([255, 255, 255], dtype=np.uint8)
lenImage = len(image)
points = []
for i in range(lenImage):
    for j in range(lenImage):
        if image[i][j][0] > 127:
            if image[i - 1][j][0] > 127:
                points.append(((i, j), (i - 1, j)))
            if image[i + 1][j][0] > 127:
                points.append(((i, j), (i + 1, j)))
            if image[i][j - 1][0] > 127:
                points.append(((i, j), (i, j - 1)))

            if j < lenImage - 1:
                if image[i][j + 1][0] > 127:
                    points.append(((i, j), (i, j + 1)))


graph = nx.Graph()
graph.add_edges_from(points)
#short = nx.dijkstra_path(graph, (1, 0), (lenImage - 2, lenImage - 1))
print(nx.dijkstra_path_length(graph, (1, 0),  (lenImage - 2, lenImage - 1)))
#print(len(short) - 1)
