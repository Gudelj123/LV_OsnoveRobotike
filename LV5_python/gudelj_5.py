import sys
import numpy as np
import vtk
import matplotlib.pyplot as plt
import matplotlib
import vtk_visualizer as vis
#import br_lectures as br
from hocook import hocook
from planecontact import planecontact
#from mobrobsim import mobrobsimanimate, set_goal, set_map
from scipy import ndimage
from PIL import Image
#from camerasim import CameraSimulator
from skimage import feature
from skimage.transform import hough_line, hough_line_peaks

# TASK 5



def findPath(nnf, start, goal, r):
    path = [[goal[0], goal[1]]]
    curr_idx = 0
    while True:
        curr_pix = path[curr_idx]
        n_indices = ((curr_pix[0] - 1, curr_pix[1]), (curr_pix[0] + 1, curr_pix[1]),
                     (curr_pix[0], curr_pix[1] - 1), (curr_pix[0], curr_pix[1] + 1))
        smallest = (goal[0], goal[1])
        for idx in n_indices:
            if idx[0] == start[0] and idx[1] == start[1]:
                path.append(idx)
                return path
            if nnf[smallest] > nnf[idx] and nnf[idx] != -1:   
                smallest = idx
        path.append(list(smallest))
        curr_idx += 1
    
    return path

def compute_nnf(start, goal, binary_map, free_space):
    envir = 15000
    new_map = np.zeros_like(binary_map)
    for row in range(len(new_map[:, 0])):
        for col in range(len(new_map[0, :])):
            if (free_space[row, col] == True):
                new_map[row, col] = envir
            else:
                new_map[row, col] = -1
                
    new_map[start[0], start[1]] = 0

    value_to_set = 1
    found = False
    while True:
        for i in range(new_map.shape[0]):
            for j in range(new_map.shape[1]):
                if new_map[i, j] != envir and new_map[i, j] != -1 and new_map[i, j] == value_to_set - 1:
                    n_indices = ((i - 1, j), (i + 1, j),
                                   (i, j - 1), (i, j + 1))
                    for idx in n_indices:
                        try:
                            if idx[0] == goal[0] and idx[1] == goal[1]:
                                new_map[idx] = value_to_set + 1
                                found = True
                                np.save("computed.npy", new_map)
                                return new_map, found
                            if new_map[idx] == envir:
                                new_map[idx] = value_to_set           
                        except:
                            pass 
        value_to_set += 1
    
    return new_map, found
    
def task5(map_file_name, start, goal, map_resolution, robot_radius):
    # Load map as image.
    map = Image.open(map_file_name)
    map = np.array(map)
    map = map[:,:,0]

    # Create binary map.
    map_mean = np.mean(map)
    binary_map = np.ones(map.shape).astype('int32')
    binary_map[map < map_mean] = 0
    plt.imshow(binary_map)
    plt.show()
    
    # Dilate obstacles.
    edt = ndimage.morphology.distance_transform_edt(binary_map)
    plt.imshow(edt)
    plt.show()
    free_space = edt > robot_radius / map_resolution
    plt.imshow(free_space)
    plt.show()

    # Compute numerical navigation function.
    # TO DO:
    print("+++++++++++++++++++")
    print("Start")
    try:
        nnf = np.load("computed.npy")
        path_found = True
    except:
        nnf, path_found = compute_nnf(start, goal, binary_map, free_space)
    
    plt.imshow(nnf)
    plt.show()
    
    # Find path.
    if path_found:
        # TO DO:
        path = findPath(nnf, start, goal, robot_radius)
        #
        map_with_path = np.ones(map.shape)
        map_with_path[map < map_mean] = 0
        path = np.array(path)
        map_with_path[path[:,0], path[:,1]] = 0.5
        plt.imshow(map_with_path)
        plt.show()
        path[:,1] = map.shape[0] - path[:,1]
        path = path.astype('float64') * map_resolution
        return path
    else:
        print('Path is not found.')
    print("End")
    print("+++++++++++++++++++")


                                
def main():
    task5('karta.png', np.array([60,360]), np.array([360,680]), 1, 2)

if __name__ == '__main__':
    main()