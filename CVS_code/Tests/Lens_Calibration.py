import numpy as np
import cv2 as cv
import glob

### identificacion de patron de ajedres ###

#chessboardSize = (24,17)
#frameSize = (1440,1080)
#
#criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30,0.001)
#
#objp = np.zeros((chessboardSize[0]*chessboardSize[1],3), np.float32)
#objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)
#
#objPoints = []
#imgPoints = []

images = glob.glob('Calib_images/*.png')

for image in images:
	print(image)
	img = cv.imread(image)
	cv.imshow("img",img)
	cv.waitKey()

cv.destroyAllWindows()
