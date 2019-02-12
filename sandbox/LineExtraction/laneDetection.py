#Stuff to do:
# Fit both f(x) and f(y) -- done
# Change regression model -- not neccessary
# Find R^2 -- done
#Fix classification of contours -- currently only the first one goes into list 2. we need to do that till we have an element in list 2 -- done
#Order/sort all contours by lowest x (figure this out pls)
#Consider comparing by x values too wihle sorting countours as left and right. 
import numpy as np
import pandas as pd
import cv2
import os
import glob
import matplotlib.pyplot as plt
import pickle
import math
from sklearn.metrics import r2_score
from sklearn.metrics import mean_squared_error
import imutils
import time



# left_a, left_b, left_c = [],[],[]
# right_a, right_b, right_c = [],[],[]
# def slide_window(binary_warped, histogram):
#     out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
#     midpoint = np.int(histogram.shape[0]/2)
#     leftx_base = np.argmax(histogram[:midpoint])
#     rightx_base = np.argmax(histogram[midpoint:]) + midpoint

#     nwindows = 9
#     window_height = np.int(binary_warped.shape[0]/nwindows)
#     nonzero = binary_warped.nonzero()
#     nonzeroy = np.array(nonzero[0])
#     nonzerox = np.array(nonzero[1])
#     leftx_current = leftx_base
#     rightx_current = rightx_base
#     margin = 100
#     minpix = 50
#     left_lane_inds = []
#     right_lane_inds = []

#     for window in range(nwindows):
#         win_y_low = binary_warped.shape[0] - (window+1)*window_height
#         win_y_high = binary_warped.shape[0] - window*window_height
#         win_xleft_low = leftx_current - margin
#         win_xleft_high = leftx_current + margin
#         win_xright_low = rightx_current - margin
#         win_xright_high = rightx_current + margin
#         cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
#         (0,255,0), 2)
#         cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
#         (0,255,0), 2)
#         good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
#         (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
#         good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
#         (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
#         left_lane_inds.append(good_left_inds)
#         right_lane_inds.append(good_right_inds)
#         if len(good_left_inds) > minpix:
#             leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
#         if len(good_right_inds) > minpix:        
#             rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

#     left_lane_inds = np.concatenate(left_lane_inds)
#     right_lane_inds = np.concatenate(right_lane_inds)

#     leftx = nonzerox[left_lane_inds]
#     lefty = nonzeroy[left_lane_inds]
#     rightx = nonzerox[right_lane_inds]
#     righty = nonzeroy[right_lane_inds]

#     left_fit = np.polyfit(lefty, leftx, 2)
#     right_fit = np.polyfit(righty, rightx, 2)

#     ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
#     left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
#     right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

#     out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
#     out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

#     plt.imshow(out_img)
#     plt.show()
#     plt.plot(left_fitx, ploty, color='yellow')
#     plt.show()
#     plt.plot(right_fitx, ploty, color='yellow')
#     plt.show()
#     plt.xlim(0, 1280)
#     plt.ylim(720, 0)

#     return ploty, left_fit, right_fit

# def skip_sliding_window(binary_warped, left_fit, right_fit):
#     nonzero = binary_warped.nonzero()
#     nonzeroy = np.array(nonzero[0])
#     nonzerox = np.array(nonzero[1])
#     margin = 100
#     left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy +
#     left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) +
#     left_fit[1]*nonzeroy + left_fit[2] + margin)))

#     right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy +
#     right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) +
#     right_fit[1]*nonzeroy + right_fit[2] + margin)))  

#     leftx = nonzerox[left_lane_inds]
#     lefty = nonzeroy[left_lane_inds]
#     rightx = nonzerox[right_lane_inds]
#     righty = nonzeroy[right_lane_inds]
#     left_fit = np.polyfit(lefty, leftx, 2)
#     right_fit = np.polyfit(righty, rightx, 2)
#     ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
#     left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
#     right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]


#     ################################
#     ## Visualization
#     ################################

#     out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
#     window_img = np.zeros_like(out_img)
#     out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
#     out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

#     left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
#     left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin,
#                                   ploty])))])
#     left_line_pts = np.hstack((left_line_window1, left_line_window2))
#     right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
#     right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin,
#                                   ploty])))])
#     right_line_pts = np.hstack((right_line_window1, right_line_window2))

#     cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
#     cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
#     result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

#     plt.imshow(result)
#     plt.show()
#     plt.plot(left_fitx, ploty, color='yellow')
#     plt.show()
#     plt.plot(right_fitx, ploty, color='yellow')
#     plt.show()
#     plt.xlim(0, 1280)
#     plt.ylim(720, 0)

#     ret = {}
#     ret['leftx'] = leftx
#     ret['rightx'] = rightx
#     ret['left_fitx'] = left_fitx
#     ret['right_fitx'] = right_fitx
#     ret['ploty'] = ploty
#     ret['left_fit'] = left_fit
#     ret['right_fit'] = right_fit

#     return ret

# def get_curve(img, leftx, rightx):
#     ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
#     y_eval = np.max(ploty)
#     ym_per_pix = 30.5/750 # meters per pixel in y dimension
#     xm_per_pix = 3.7/750 # meters per pixel in x dimension

#     # Fit new polynomials to x,y in world space
#     left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
#     right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)
#     # Calculate the new radii of curvature
#     left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
#     right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])

#     car_pos = img.shape[1]/2
#     l_fit_x_int = left_fit_cr[0]*img.shape[0]**2 + left_fit_cr[1]*img.shape[0] + left_fit_cr[2]
#     r_fit_x_int = right_fit_cr[0]*img.shape[0]**2 + right_fit_cr[1]*img.shape[0] + right_fit_cr[2]
#     lane_center_position = (r_fit_x_int + l_fit_x_int) /2
#     center = (car_pos - lane_center_position) * xm_per_pix / 10
#     # Now our radius of curvature is in meters
#     return (left_curverad, right_curverad, center)

# def get_hist(img):
#     hist = np.sum(img[img.shape[0]//2:,:], axis=0)
#     return hist

# def inv_perspective_warp(img, 
#                      dst_size=(1349,750),
#                      src=np.float32([(0,0), (1, 0), (0,1), (1,1)]),
#                      dst=np.float32([(0.43,0.65),(0.58,0.65),(0.1,1),(1,1)])):
#     img_size = np.float32([(img.shape[1],img.shape[0])])
#     src = src* img_size
#     # For destination points, I'm arbitrarily choosing some points to be
#     # a nice fit for displaying our warped result 
#     # again, not exact, but close enough for our purposes
#     dst = dst * np.float32(dst_size)
#     # Given src and dst points, calculate the perspective transform matrix
#     M = cv2.getPerspectiveTransform(src, dst)
#     # Warp the image using OpenCV warpPerspective()
#     warped = cv2.warpPerspective(img, M, dst_size)
#     return warped

# def draw_lanes(img, left_fit, right_fit):
#     ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
#     color_img = np.zeros_like(img)
	
#     left = np.array([np.transpose(np.vstack([left_fit, ploty]))])
#     right = np.array([np.flipud(np.transpose(np.vstack([right_fit, ploty])))])
#     points = np.hstack((left, right))
	
#     cv2.fillPoly(color_img, np.int_(points), (0,200,255))
#     inv_perspective = inv_perspective_warp(color_img)
#     inv_perspective = cv2.addWeighted(img, 1, inv_perspective, 0.7, 0)
#     return inv_perspective

# def perspective_warp(img, 
#                      dst_size=(1349,750),
#                      src=np.float32([(0.43,0.65),(0.58,0.65),(0.1,1),(1,1)]),
#                      dst=np.float32([(0,0), (1, 0), (0,1), (1,1)])):
#     img_size = np.float32([(img.shape[1],img.shape[0])])
#     src = src* img_size
#     # For destination points, I'm arbitrarily choosing some points to be
#     # a nice fit for displaying our warped result 
#     # again, not exact, but close enough for our purposes
#     dst = dst * np.float32(dst_size)
#     # Given src and dst points, calculate the perspective transform matrix
#     M = cv2.getPerspectiveTransform(src, dst)
#     # Warp the image using OpenCV warpPerspective()
#     warped = cv2.warpPerspective(img, M, dst_size)
#     return warped
# im = cv2.imread('images/image4.png')
class contourObject:

	def __init__(self, contour, extLeft, extRight, midpoint):
		self.contour = contour
		self.extLeft = extLeft
		self.extRight = extRight
		self.midpoint = midpoint 

im = cv2.imread("images/image7.png")
t0 = time.time()
height = np.size(im, 0)
width = np.size(im, 1)
# plt.imshow(im)
# plt.show()
imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

# kernel = np.ones((5,5), np.uint8)
# img_dilation = cv2.dilate(imgray, kernel, iterations=4)
# # cv2.imshow('', img_dilation)
# # cv2.waitKey(0)
# img_erosion = cv2.erode(img_dilation, kernel, iterations=3)
# cv2.imshow('', img_erosion)
# cv2.waitKey(0)
ret, thresh = cv2.threshold(imgray, 127, 255, 0)
im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# for cnt in contours:
#     cv2.drawContours(im,[cnt],0,(0,255,0),2)

# cv2.imshow('output',im)
# cv2.waitKey(0)

x = 0;
noElementInList2 = True


array1 = []
array2 = []


arr1XMid = 0
arr1YMid = 0
arr1ExtLeft = 0#tuple(c[c[:, :, 0].argmin()][0])
arr1ExtRight = 0#tuple(c[c[:, :, 0].argmax()][0])
arr2XMid = 0
arr2YMid = 0
arr2ExtLeft = 0#tuple(c[c[:, :, 0].argmin()][0])
arr2ExtRight = 0#tuple(c[c[:, :, 0].argmax()][0])


array1x = []
array1y = []
array2x = []
array2y = []

# contour_min_xs = []
# valid_contours = []

def sort_contours(cnts, method="left-to-right"):
	# initialize the reverse flag and sort index
	reverse = False
	i = 0
 
	# handle if we need to sort in reverse
	if method == "right-to-left" or method == "bottom-to-top":
		reverse = True
 
	# handle if we are sorting against the y-coordinate rather than
	# the x-coordinate of the bounding box
	if method == "top-to-bottom" or method == "bottom-to-top":
		i = 1
 
	# construct the list of bounding boxes and sort them from top to
	# bottom
	boundingBoxes = [cv2.boundingRect(c) for c in cnts]
	(cnts, boundingBoxes) = zip(*sorted(zip(cnts, boundingBoxes),
		key=lambda b:b[1][i], reverse=reverse))
 
	# return the list of sorted contours and bounding boxes
	return (cnts, boundingBoxes)

boundingBoxes = [cv2.boundingRect(c) for c in contours]
(contours, boundingBoxes) = zip(*sorted(zip(contours, boundingBoxes),
	key=lambda b:b[1][0], reverse=False))
# for i in range(len(contours)):
#     if cv2.arcLength(contours[i], True) < 67 or cv2.contourArea(contours[i]) < 34.0:
#         continue
#     valid_contours.append(contours[i])
# for i in range(len(valid_contours)):
#     smallest_x = im.shape[1]    
#     for j in range(len(contours[i])):
#         x = contours[i][j][0][0]
#         print(x)
#         if x < smallest_x:
#             smallest_x = x
#     contour_min_xs.append(smallest_x)
# inds = np.argsort(contour_min_xs)
# contours = list(np.array(contours)[inds])

for cnt in contours:
	# if cv2.arcLength(cnt, True) < 67 or cv2.contourArea(cnt) < 34.0:
	#  	continue
	currExtLeft = tuple(cnt[cnt[:, :, 0].argmin()][0])
	currExtRight = tuple(cnt[cnt[:, :, 0].argmax()][0])
	xVal = 0
	yVal = 0
	points = 0
	for pointA in cnt:
		xVal = xVal + pointA[0][0]
		yVal = yVal + pointA[0][1]
		points = points+1
	xVal = xVal/points
	yVal = yVal/points
	cv2.circle(im, (xVal, yVal), 7, (255, 255, 255), -1)
	if x == 0:
		# array1.append(cnt)
		# arr1XMid = xVal
		# arr1YMid = yVal
		# arr1ExtRight = currExtRight
		# arr1ExtLeft = currExtLeft
		array1.append(contourObject(cnt, currExtLeft, currExtRight, [xVal, yVal]))

		# cv2.drawContours(im, [cnt],0,(0,255,0),2)
		# array1.append([cnt])
	else:
		if noElementInList2:
			# print(((arr1XMid - xVal) ** 2 + (arr1YMid - yVal) ** 2) ** 0.5)
			lowestDist = float("inf")
			for cntObj in array1:
				distListArr1 = [((cntObj.midpoint[0] - xVal) ** 2 + (cntObj.midpoint[1] - yVal) ** 2) ** 0.5 , 
				((cntObj.extRight[0] - currExtLeft[0]) ** 2 + (cntObj.extRight[1] - currExtLeft[1]) ** 2) ** 0.5, 
				((cntObj.extLeft[0] - currExtRight[0]) ** 2 + (cntObj.extLeft[1] - currExtRight[1]) ** 2) ** 0.5]
				lowestDist = min([lowestDist, min(distListArr1)])
			#print(min(distListArr1))
			#print(((arr1ExtRight[0] - currExtLeft[0]) ** 2 + (arr1ExtRight[1] - currExtLeft[1]) ** 2) ** 0.5)
			#print(arr1ExtRight)
			
			if  lowestDist > 63:
				# print(((arr1XMid - xVal) ** 2 + (arr1YMid - yVal) ** 2) ** 0.5)
				array2.append(contourObject(cnt, currExtLeft, currExtRight, [xVal, yVal]))
				# arr2XMid = xVal
				# arr2YMid = yVal
				# arr2ExtRight = currExtRight
				# arr2ExtLeft = currExtLeft
				noElementInList2 = False
			else:
				array1.append(contourObject(cnt, currExtLeft, currExtRight, [xVal, yVal]))
				# arr1XMid = xVal
				# arr1YMid = yVal
				# arr1ExtRight = currExtRight
				# arr1ExtLeft = currExtLeft
		else:

			lowestDist1 = float("inf")
			for cntObj in array1:
				distListArr1 = [((cntObj.midpoint[0] - xVal) ** 2 + (cntObj.midpoint[1] - yVal) ** 2) ** 0.5 , 
				((cntObj.extRight[0] - currExtLeft[0]) ** 2 + (cntObj.extRight[1] - currExtLeft[1]) ** 2) ** 0.5, 
				((cntObj.extLeft[0] - currExtRight[0]) ** 2 + (cntObj.extLeft[1] - currExtRight[1]) ** 2) ** 0.5]
				lowestDist1 = min([lowestDist1, min(distListArr1)])
			# distListArr1 = [((arr1XMid - xVal) ** 2 + (arr1YMid - yVal) ** 2) ** 0.5 , 
			# ((arr1ExtRight[0] - currExtLeft[0]) ** 2 + (arr1ExtRight[1] - currExtLeft[1]) ** 2) ** 0.5, 
			# ((arr1ExtLeft[0] - currExtRight[0]) ** 2 + (arr1ExtLeft[1] - currExtRight[1]) ** 2) ** 0.5]


			lowestDist2 = float("inf")
			for cntObj in array2:
				distListArr2 = [((cntObj.midpoint[0] - xVal) ** 2 + (cntObj.midpoint[1] - yVal) ** 2) ** 0.5 , 
				((cntObj.extRight[0] - currExtLeft[0]) ** 2 + (cntObj.extRight[1] - currExtLeft[1]) ** 2) ** 0.5, 
				((cntObj.extLeft[0] - currExtRight[0]) ** 2 + (cntObj.extLeft[1] - currExtRight[1]) ** 2) ** 0.5]
				lowestDist2 = min([lowestDist2, min(distListArr2)])
			# distListArr2 = [((arr2XMid - xVal) ** 2 + (arr2YMid - yVal) ** 2) ** 0.5 , 
			# ((arr2ExtRight[0] - currExtLeft[0]) ** 2 + (arr2ExtRight[1] - currExtLeft[1]) ** 2) ** 0.5, 
			# ((arr2ExtLeft[0] - currExtRight[0]) ** 2 + (arr2ExtLeft[1] - currExtRight[1]) ** 2) ** 0.5]
			# print(((arr1XMid - xVal) ** 2 + (arr1YMid - yVal) ** 2) ** 0.5)

			if lowestDist1 > lowestDist2:
				array2.append(contourObject(cnt, currExtLeft, currExtRight, [xVal, yVal]))
				# arr2XMid = xVal
				# arr2YMid = yVal
				# arr2ExtRight = currExtRight
				# arr2ExtLeft = currExtLeft
				# print("Here")
			else:
				array1.append(contourObject(cnt, currExtLeft, currExtRight, [xVal, yVal]))
				# arr1XMid = xVal
				# arr1YMid = yVal
				# arr1ExtRight = currExtRight
				# arr1ExtLeft = currExtLeft

	x = x + 1

# cv2.drawContours(im, array1[0],0,(0,255,0),2)
# cv2.drawContours(im, array2[0],0,(255,0,0),2)
# print(array1)
# print(array2)
# cv2.imshow("", im)
# cv2.waitKey(0);

# nearest_dist = 100000000000000000
# print(contours[0][0].size)
# xVal = 0
# yVal = 0
# for pointA in contours[1]:
# 	xVal = xVal + pointA[0][0]
# 	yVal = yVal + pointA[0][1]
# print(xVal/887)
# print(yVal/887)
# print(array1)
# print(len(array1))
# print(len(array2))
# THIS WORKS
for cntObj in array1:
	cv2.drawContours(im, [cntObj.contour],0,(255,0,0),2)
	# print(cnt.size)
	for pointA in cntObj.contour:
		array1x.append(pointA[0][0])
		array1y.append(pointA[0][1])
		# print(cnt)
		# array1.append([cnt])
for cntObj in array2:
	cv2.drawContours(im, [cntObj.contour],0,(0,0,255),2)
	for pointA in cntObj.contour:
		array2x.append(pointA[0][0])
		array2y.append(pointA[0][1])
		# print(cnt)
		# array1.append([cnt])




# cv2.drawContours(im, [contours[3]],0,(0,255,0),2)
# cv2.imshow("", im)
# cv2.waitKey(0);

# print(len(array1x))
# print(array1y)
l_funcx_points = []
l_funcx_ypred = []

l_funcy_points = []
l_funcy_xpred = []

r_funcx_points = []
r_funcx_ypred = []

r_funcy_points = []
r_funcy_xpred = []


# if len(array1x) > 0:
# 	#array1x = array1x.reshape(-1, 1)
# 	model = make_pipeline(PolynomialFeatures(2), Ridge())
# 	model = model.fit(array1x, array1y)
# 	print(model)
# 	model.named_steps['linear'].coef_
# 	y_plot = model.predict(array1x)
# 	print(y_plot)


if len(array1x) > 0:

	#fit y = x^2
	l = np.polyfit(array1x, array1y, 2)  
	x1 = np.linspace(0, width, 400)
	for i in x1:
		# y1 = l[0] * i**5 + l[1] * i**4 + l[2] * i**3 + l[3] * i**2 + l[4] * i + l[5] 
		y1 = l[0] * i**1 + l[1] * i**1 + l[2]
		# y1 = l[0] * i + l[1]
		if y1 < height and y1 > 0:
			l_funcx_points.append([i, y1]) 
	for i in array1x:
		y1 = l[0] * i**2 + l[1] * i**1 + l[2]
		# y1 = l[0] * i**5 + l[1] * i**4 + l[2] * i**3 + l[3] * i**2 + l[4] * i + l[5]
		# y1 = l[0] * i + l[1]
		l_funcx_ypred.append(y1)


	#fit x = y^2 -- for hairpin
	l = np.polyfit(array1y, array1x, 2)  #Switched for hairpin
	y1 = np.linspace(0, height, 400)
	for i in y1:
		x1 = l[0] * i**2 + l[1] * i**1 + l[2]
		# x1 = l[0] * i**5 + l[1] * i**4 + l[2] * i**3 + l[3] * i**2 + l[4] * i + l[5]
		# x1 = l[0] * i + l[1]
		if x1 < width and x1 > 0:
			l_funcy_points.append([x1, i]) #Switched for hairpin
	for i in array1y:
		x1 = l[0] * i**2 + l[1] * i**1 + l[2]
		# x1 = l[0] * i**5 + l[1] * i**4 + l[2] * i**3 + l[3] * i**2 + l[4] * i + l[5]
		# x1 = l[0] * i + l[1]
		l_funcy_xpred.append(x1)

	# print(array1y)
	# print(l_funcx_ypred)
	# print(mean_squared_error(array1y, l_funcx_ypred))
	# print(mean_squared_error(array1x, l_funcy_xpred))
	# l_points = np.array(l_funcx_points, dtype=np.int32)
	# cv2.polylines(im, [l_points], 0, (255,0,0))
	# l_points = np.array(l_funcy_points, dtype=np.int32)
	# cv2.polylines(im, [l_points], 0, (0,255,0))

	if mean_squared_error(array1y, l_funcx_ypred) < mean_squared_error(array1x, l_funcy_xpred):
		l_points = np.array(l_funcx_points, dtype=np.int32)
		cv2.polylines(im, [l_points], 0, (255,0,0))
	else:
		l_points = np.array(l_funcy_points, dtype=np.int32)
		cv2.polylines(im, [l_points], 0, (255,0,0))


if len(array2x) > 0:

	#fit y = x^2
	r = np.polyfit(array2x, array2y, 2)
	x2 = np.linspace(0, width, 400)
	for i in x2:
		y2 = r[0] * i**2 + r[1] * i**1 + r[2]
		# y2 = r[0] * i**5 + r[1] * i**4 + r[2] * i**3 + r[3] * i**2 + r[4] * i + r[5]
		# y2 = r[0] * i + r[1]
		if y2 < height and y2 > 0:
			r_funcx_points.append([i, y2])
	for i in array2x:
		y2 = r[0] * i**2 + r[1] * i**1 + r[2]
		# y2 = r[0] * i**5 + r[1] * i**4 + r[2] * i**3 + r[3] * i**2 + r[4] * i + r[5]
		# y2 = r[0] * i + r[1]
		r_funcx_ypred.append(y2)

	#fit x = y^2
	r = np.polyfit(array2y, array2x, 2)
	y2 = np.linspace(0, width, 400)
	for i in y2:
		x2 = r[0] * i**2 + r[1] * i**1 + r[2]
		# x2 = r[0] * i**5 + r[1] * i**4 + r[2] * i**3 + r[3] * i**2 + r[4] * i + r[5]
		# x2 = r[0] * i + r[1]
		if x2 < width and x2 > 0:
			r_funcy_points.append([x2, i])
	for i in array2y:
		x2 = r[0] * i**2 + r[1] * i**1 + r[2]
		# x2 = r[0] * i**5 + r[1] * i**4 + r[2] * i**3 + r[3] * i**2 + r[4] * i + r[5]
		# x2 = r[0] * i + r[1]
		r_funcy_xpred.append(x2)

	# r_points = np.array(r_funcx_points, dtype=np.int32)
	# cv2.polylines(im, [r_points], 0, (0,0,255))
	# r_points = np.array(r_funcy_points, dtype=np.int32)
	# cv2.polylines(im, [r_points], 0, (0,0,255))

	if mean_squared_error(array2y, r_funcx_ypred) < mean_squared_error(array2x, r_funcy_xpred):
		r_points = np.array(r_funcx_points, dtype=np.int32)
		cv2.polylines(im, [r_points], 0, (0,0,255))
	else:
		r_points = np.array(r_funcy_points, dtype=np.int32)
		cv2.polylines(im, [r_points], 0, (0,0,255))



# print(len(l_points[0][0]))
t1 = time.time()
print(t1 - t0)
cv2.imshow("", im)
cv2.waitKey(0);
# fig, ax = plt.subplots()

# ax.imshow(im, extent=[0, 1280, 0, 964])
# img = plt.imread("test3.png")
# fig, ax = plt.subplots()
# ax.plot(x, z[0] * x**3 + z[1] * x**2 + z[2] * x**1 + z[3])
#z[0] * x**2 + z[1] * x**1 + z[2]


# fig, ax = plt.subplots()
# x1 = np.linspace(1, 2, 1000)
# x2 = np.linspace(99, 100, 1000)
# ax.imshow(im)
# if len(array1x) > 0:
# 	l = np.polyfit(array1x, array1y, 3)
# 	ax.plot(x1, l[0] * x1**3 + l[1] * x1**2 + l[2] * x1**1 + l[3], '--', linewidth=5, color='green')
# if len(array2x) > 0:
# 	r = np.polyfit(array2x, array2y, 3)
# 	ax.plot(x2, r[0] * x2**3 + r[1] * x2**2 + r[2] * x2**1 + r[3], '--', linewidth=5, color='blue')
# plt.show()

# cv2.imshow("", image)
# cv2.waitKey(0)

# cv2.imshow("", image)
# cv2.waitKey(0)

# #print(image)
# # im = cv2.imshow("",im_gray)
# # cv2.waitKey(0)
# # print(im_gray)


# img = inv_perspective_warp(im_bw)
# histogram = get_hist(im_bw)
# plt.plot(histogram)
# plt.show()
# cv2.imshow("", img)
# cv2.waitKey(0)
# ploty, left_fit, right_fit = slide_window(im_bw, histogram)
# ret = skip_sliding_window(im_bw, left_fit, right_fit)
# print(ret['left_fit'])
# cv2.imshow("", out_img)
# cv2.waitKey(0)
# print(lanes)
# plt.plot(curves[0], ploty, color='yellow', linewidth=1)
# # #plt.plot(curves[1], ploty, color='yellow', linewidth=1)
# # print(np.asarray(curves).shape)
# # #img = inv_perspective_warp(im_bw)
# curverad=get_curve(img, curves[0],curves[1])
# # print(curverad)
# img_ = draw_lanes(im_gray, curves[0], curves[1])
# # cv2.imshow("", img_)
# cv2.waitKey(0)
