import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
from pyefd import elliptic_fourier_descriptors
from pyefd import normalize_efd
# source: https://docs.opencv.org/master/d4/d13/tutorial_py_filtering.html

#THIS SCRIPT IS A STANDALONE TEST BED FOR THE IMAGE PROCESSING
#THE FINAL, COMPLETE SCRIPT WILL BE FRAMED INTO ROS THROUGH THE image_processor.py NODE
#AS SUCH, NO MODIFICATIONS TO CMakeLists.txt or package.xml ARE NECESSARY

# load in image
#img = cv.imread('GLASS-RED_Engraved.png',0) #other images for testing
#img = cv.imread('calli.png',0)
img = cv.imread('Hourglass_Solid.jpg') #only image that actually produces single, continuous edge


# Canny edge detector which produces the binarized image containing only the silhouette ideally
edges = cv.Canny(img,100,200) #(100,200) was default, (20,90) produced good berk, (600,700) produced good wine glass



# THIS SECTION OF COMMENTED CODE IS ONLY FOR DUMMY THICCENING THE IMAGE TO CLOSE GAPS
#kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(9,9))
#dilated = cv.dilate(edges, kernel)
#_, cnts, _ = cv.findContours(dilated.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

#np.isfinite(dilated).all()

#coeffs = elliptic_fourier_descriptors(dilated, order=10)



# THE ACTUAL CALCULATION OF THE EFD COEFFICIENTS IS HERE 

# find the contours of a binary image using OpenCV
contours = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

#iterate through all contours found and store each contour's 
#eliiptical Fourier descriptor's (EFD's) coefficients.
coeffs = []
for cnt in contours:
    #find the coefficients of all contours
    cnt = np.array(tuple(cnt))
    coeffs.append(elliptic_fourier_descriptors(np.squeeze(cnt), order=10))

print(coeffs)

plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
#plt.subplot(122),plt.imshow(dilated,cmap = 'gray') # used only for Dummy Thiccening plot
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

plt.show()

