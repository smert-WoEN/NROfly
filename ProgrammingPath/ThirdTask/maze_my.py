import cv2
import numpy as np
import requests

def get_neighbours(img,vis, c):
    neighbours = []
    if c[0] > 0 and not vis[c[0]-1,c[1]]:
        neighbours.append(np.array([c[0]-1,c[1]]))
    if c[0] < img.shape[0]-1 and not vis[c[0]+1,c[1]]:
        neighbours.append(np.array([c[0]+1,c[1]]))
    if c[1] > 0 and not vis[c[0],c[1]-1]:
        neighbours.append(np.array([c[0],c[1]-1]))
    if c[1] < img.shape[1]-1 and not vis[c[0],c[1]+1]:
        neighbours.append(np.array([c[0],c[1]+1]))
    return np.array(neighbours)

def diff(img, a, b):
    return np.sum((img[a[0], a[1]].astype(int)-img[b[0], b[1]].astype(int))**2) + 1

def find_min(dist, vis):
    non_vis = np.nonzero(~vis)
    ind = np.argmin(dist[non_vis[0],non_vis[1]])
    return np.array([non_vis[0][ind], non_vis[1][ind]])

def find_min_naive(dist, vis):
    non_vis = np.nonzero(~vis)
    current_min_node = None
    for node_id in range(non_vis[0].shape[0]): # Iterate over the nodes
        if current_min_node == None:
            current_min_node = node_id
        elif dist[non_vis[0][node_id],non_vis[1][node_id]] < dist[non_vis[0][current_min_node],non_vis[1][current_min_node]]:
            current_min_node = node_id
    return np.array([non_vis[0][current_min_node],non_vis[1][current_min_node]])

def find_opt_dist(img,start,end):
    start_x, start_y = start
    end_x, end_y = end

    dist_mat = np.zeros((img.shape[0],img.shape[1])) + np.inf
    dist_mat[start_x,start_y] = 0
    visited_mat = np.zeros((img.shape[0],img.shape[1]),dtype=bool)
    while np.count_nonzero(~visited_mat) != 0:
        cell = find_min(dist_mat,visited_mat)
        neigh = get_neighbours(img, visited_mat, cell)
        for v in neigh:
            if dist_mat[v[0], v[1]] > dist_mat[cell[0],cell[1]] + diff(img,cell,v):
                dist_mat[v[0], v[1]] = dist_mat[cell[0],cell[1]] + diff(img,cell,v)
        visited_mat[cell[0],cell[1]] = True
        if cell[0] == end_x and cell[1] == end_y:
            print(image[visited_mat & (image[:,:,0] == 255)].shape)
            # for i in range(10):
            #     print(dist_mat[i,:10])
            #     print()
            # print(get_neighbours(image,visited_mat,cell))
            non_vis = np.nonzero(visited_mat)
            for node_id in range(non_vis[0].shape[0]):
                image[non_vis[0][node_id],non_vis[1][node_id]][0] = 255
                image[non_vis[0][node_id],non_vis[1][node_id]][1] = 0
                image[non_vis[0][node_id],non_vis[1][node_id]][2] = 0

            cv2.imshow("",img)
            while True:
                key = cv2.waitKey(0)
                if key == ord('q'):
                    break
            return dist_mat[cell[0],cell[1]]
    
    return dist_mat[end_x,end_y]


if __name__ == "__main__":
    url = input()
    resp = requests.get(url, stream=True).raw
    image = np.asarray(bytearray(resp.read()), dtype=np.uint8)
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    dist = find_opt_dist(image,(1,0),(image.shape[0]-2,image.shape[1]-1))
    
    print(int(dist))
