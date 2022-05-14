import os
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

def find_corners(img_path):
    img = cv.imread(img_path)
    rgb_img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    hsv_img = cv.cvtColor(rgb_img,cv.COLOR_RGB2HSV)
    gray = cv.cvtColor(rgb_img,cv.COLOR_BGR2GRAY)

    light_orange = (7, 105, 70)
    dark_orange = (23, 245, 185)

    mask = cv.inRange(hsv_img, light_orange, dark_orange)
    result = cv.bitwise_and(rgb_img, rgb_img, mask=mask)
    plt.imshow(result)
    plt.show()

    blur = cv.blur(result,(3,3),0)
    gray_blur = cv.cvtColor(blur,cv.COLOR_BGR2GRAY) 

    gray_blur[gray_blur>7]=255
    gray_blur = cv.medianBlur(gray_blur,7)
    plt.imshow(gray_blur)
    plt.show()

    resultnp = np.float32(gray_blur)
    dst1 = cv.cornerHarris(resultnp,2,7,0.01)
    gate = dst1.copy()
    threshold = 0.1
    gate[gate<threshold*dst1.max()]=0
    gate[gate>threshold*dst1.max()]=1
    plt.imshow(gate)
    plt.show()

    corners = np.transpose(np.nonzero(gate))
    x_corners = corners[:,1]
    y_corners = corners[:,0]
    x_max = x_corners.max()
    x_min = x_corners.min()
    distance = x_max-x_min
    limit = distance/15
    print(distance,limit)

    gate_corners = []
    for corner in corners:
        y = corner[0]
        x = corner[1]
        if x < x_min + limit:
            gate_corners.append(corner)
        elif x > x_max - limit:
            gate_corners.append(corner)

    gate_corners = np.array(gate_corners)
    rgb_img1 = rgb_img.copy()
    for center in gate_corners:
        x = int(center[0])
        y = int(center[1])
        rgb_img1[x-1:x+1,y-1:y+1] = [255,0,0]
    plt.imshow(rgb_img1)
    plt.show()

    kmeans = KMeans(
          init="random",
          n_clusters=4,
          n_init=5,
          max_iter=300,
          random_state=42
    )
    kmeans.fit(gate_corners)
    centers = kmeans.cluster_centers_
    rgb_img2 = rgb_img.copy()
    for center in centers:
        x = int(center[0])
        y = int(center[1])
        rgb_img2[x-2:x+2,y-2:y+2] = [255,0,0]
    plt.imshow(rgb_img2)
    plt.show()

directory = "images"
for f in os.listdir(directory):
    filename = os.path.join(directory,f)
    find_corners(filename)
