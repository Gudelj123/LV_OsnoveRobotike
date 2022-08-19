import numpy as np
import matplotlib.pyplot as plt
from camerasim import CameraSimulator
from skimage import feature
from skimage.transform import hough_line, hough_line_peaks
import cv2 

# TASK 4

def imgproc(RGB, camera, camera_height, box_size):
    img_w = camera['img_size'][0]
    img_h = camera['img_size'][1]
    box_w = box_size[0]
    box_h = box_size[1]
    E=feature.canny(RGB[:,:,0])    
    H, theta, d = hough_line(E)    
    for _, angle, dist in zip(*hough_line_peaks(H, theta, d, min_distance=20, min_angle=20, threshold=50)):
        #print('angle=%f, dist=%f' % (angle, dist))
        cs = np.cos(angle)
        sn = np.sin(angle)
        if np.abs(cs) >= np.abs(sn):
            v = np.array([0, img_h-1])
            u = (dist - sn * v) / cs
        else:
            u = np.array([0, img_w-1])
            v = (dist - cs * u) / sn
        plt.plot(u, v, 'g')    

        #####     
    # convert image to grayscale image
    image = np.array(RGB, dtype=np.int16)
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("GrayImage",gray_image)

    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(gray_image,127,255,0)

    # calculate moments of binary image
    M = cv2.moments(thresh)

    # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    # put text and highlight the center
    cv2.circle(image, (cX, cY), 5, (255, 255, 255), -1)
    cv2.putText(image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # display the image
    cv2.imshow("Image", image)
    cv2.waitKey(0)       
    #Compute alpha:
    alpha = 0
    
    ######
    # Visualize red line representing orientation:

    # plt.plot(t_img[0],t_img[1],'r+')
    # x_axis = np.stack((t_img, t_img + x * 150), 0)
    # plt.plot(x_axis[:,0], x_axis[:,1], 'r-')    
    # plt.show()

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
    
    print('orientation error: %f deg position error: %f mm' % (np.rad2deg(e_alpha), 1000 * e_t))    
    
def main():
    #task0()
    #task1([0, np.pi/2, -np.pi/2, 0, 0, 0])
    #task2([0, 1, 0])
    #task3()
    task4()

if __name__ == '__main__':
    main()