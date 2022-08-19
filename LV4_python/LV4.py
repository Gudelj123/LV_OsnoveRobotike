import sys
import numpy as np
import vtk
import matplotlib.pyplot as plt
import vtk_visualizer as vis
#import br_lectures as br
from hocook import hocook
from planecontact import planecontact
from mobrobsim import mobrobsimanimate, set_goal, set_map
from scipy import ndimage
from PIL import Image
from camerasim import CameraSimulator
from skimage import feature
from skimage.transform import hough_line, hough_line_peaks
from sympy import Point, Line, Segment


# TASK 4
#+++++++++++Make line from 2 points (u,v)
def makeLine(p1, p2):
    A = p1[1] - p2[1]
    B = p2[0] - p1[0]
    C = p1[0]*p2[1] - p2[0]*p1[1]
    return A, B, -C

#+++++++++++Intersection between two lines
def intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
 
    # Method used to display X and Y coordinates
    # of a point
    def displayPoint(self, p):
        print(f"({p.x}, {p.y})")
        
def lineLineIntersection(A, B, C, D):
    # Line AB represented as a1x + b1y = c1
    a1 = B.y - A.y
    b1 = A.x - B.x
    c1 = a1*(A.x) + b1*(A.y)
 
    # Line CD represented as a2x + b2y = c2
    a2 = D.y - C.y
    b2 = C.x - D.x
    c2 = a2*(C.x) + b2*(C.y)
 
    determinant = a1*b2 - a2*b1
 
    if (determinant == 0):
        # The lines are parallel. This is simplified
        # by returning a pair of FLT_MAX
        return Point(10**9, 10**9)
    else:
        x = (b2*c1 - b1*c2)/determinant
        y = (a1*c2 - a2*c1)/determinant
        return Point(x, y)
    
def imgproc(RGB, camera, camera_height, box_size):
    img_w = camera['img_size'][0]
    img_h = camera['img_size'][1]
    box_w = box_size[0]
    box_h = box_size[1]
    plt.imshow(RGB)
    E=feature.canny(RGB[:,:,0])    
    H, theta, d = hough_line(E)    
    colors = ['r','g','b','w','m','y','k','c']
    counter = 0
    linesOfItem = [None]*4
    intersections = [None]*4
    allPoints = [None]*16
    cnt = 0
    points = [None]*8
    
    for _, angle, dist in zip(*hough_line_peaks(H, theta, d, min_distance=20, min_angle=20, threshold=50)):
        
        
        #print('angle=%f, dist=%f' % (angle, dist))
        cs = np.cos(angle)
        sn = np.sin(angle)
        if np.abs(cs) >= np.abs(sn):
            v = np.array([0, img_h-1])
            u = (dist - sn * v) / cs
            plt.plot(u, v,colors[counter]) 
        else:
            u = np.array([0, img_w-1])
            v = (dist - cs * u) / sn
            plt.plot(u, v, colors[counter])
        allPoints[cnt] = u
        allPoints[cnt+1]= v
        if(counter > 3):
            points[counter - 4] = Point(u[0],u[1])
            points[counter] = Point(v[0],v[1])
            
            linesOfItem[counter-4] = makeLine(u,v)
        counter = counter + 1
        cnt = cnt + 2
        #print(u,v)
        
    
    print(allPoints[9])
    
    
 #   for i in range (len(intersections)):
        #plt.scatter(intersections[i][0],intersections[i][1] , s=50)
    plt.show()

    
    print("------------------------------")
    print(intersections)
    print("------------------------------")     
    print(linesOfItem)    
    #plt.show(RGB)
        #####            
    #Compute alpha:
    alpha = 0
    ######
    # Visualize red line representing orientation:

    #plt.plot(t_img[0],t_img[1],'r+')
    #x_axis = np.stack((t_img, t_img + x * 150), 0)
    #plt.plot(x_axis[:,0], x_axis[:,1], 'r-')    
    #plt.show()

    ######

    #Compute t:
    t=[0,0,0]

    ######
    return alpha, t

        

def task4():
    camera = {
        'img_size': [640, 480],
        'focal_length': 1000,
        'principal_point': [320, 240],
        }
    camera_height = 0.5
    box_w = 0.1
    box_h = 0.05
    
    camsim = CameraSimulator(camera, camera_height, [box_w, box_h], 0.05)

    RGB = camsim.get_image()
    
    

    alpha, t = imgproc(RGB, camera, camera_height, [box_w, box_h])
    
    e_alpha, e_t = camsim.evaluate(alpha, t)
    
    #Centar camere
    #cv2.circle(RGB,(320, 240),50,(255,255,255),3)
    
    #plt.imshow(RGB)
    
    print('orientation error: %f deg position error: %f mm' % (np.rad2deg(e_alpha), 1000 * e_t))    
                
def main():
    #task0()
    #task1([0, np.pi/2, -np.pi/2, 0, 0, 0])
    #task2([0, 1, 0])
    #task3()
    task4()

if __name__ == '__main__':
    main()