import cv2
import sys
import numpy as np
import serial,time
import time

#Setup the camera
camera = cv2.VideoCapture(0)

#Setup the Serial Communication to Arduino
#arduino = serial.Serial('COM4', 9600, timeout=0.1)

# Setup BlobDetector
detector = cv2.SimpleBlobDetector_create()
detector1 = cv2.SimpleBlobDetector_create()
params = cv2.SimpleBlobDetector_Params()
params1 = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 5
params.maxThreshold = 200
params1.minThreshold = 5
params1.maxThreshold = 20	
 
# Filter by Area.
params.filterByArea = True
params.minArea = 1000
params.maxArea = 50000
params1.filterByArea = True
params1.minArea = 50
params1.maxArea = 1000

# Filter by Color.
params.filterByColor = True
params.blobColor = 255
params1.filterByColor = True
params1.blobColor = 0
	 
# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.5
params1.filterByCircularity = True
params1.minCircularity = 0.7
 
# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.1
params1.filterByConvexity = True
params1.minConvexity = 0.7

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.5
params1.filterByInertia = True
params1.minInertiaRatio = 0.5

# Distance Between Blobs
params.minDistBetweenBlobs = 10
params1.minDistBetweenBlobs = 10
	 
# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)
detector1 = cv2.SimpleBlobDetector_create(params1)

hitung=0
ok=0
ng=0
okflag= False
ngflag= False
ngflag1= False
trigger = None
kondisi = None
statusin = None
statusok = None
statusng = None
statusnowng = None
statusng1 = None
statusnowng1 = None
statusnow = None
lingkaran = ""
start = 0.0
end = 0.0
elapsed = 0.0
speed = 0.0
x1=0
y1=0
w1=0
h1=0



i1=0







while camera.isOpened():
	
	
	retval, im = camera.read()

	# setup initial location of window
	r, h, c, w = 0, 480, 0, 450  # simply hardcoded the values
	track_window = (c, r, w, h)

	# set up the ROI for tracking
	roiv = im[r:r + h, c:c + w]
	roi2=im.copy()
	hsv_roi = cv2.cvtColor(roiv, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv_roi, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
	roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
	cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)

	# Draw it on image
	x, y, w, h = track_window

	
	dst = np.zeros_like(im)
	dst[y:y+h,x:x+w] = im[y:y+h,x:x+w]
	#cv2.imshow('dst',dst)
	

	# Our operations on the frame come here
	gray = cv2.cvtColor(roiv, cv2.COLOR_BGR2GRAY)
	#cv2.imshow('gray',gray)
	blur= cv2.medianBlur(gray,5)
	#cv2.imshow('blur',blur)

	ret,thresh = cv2.threshold(blur,190,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

	#Draw outer Contour
	image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	overlay = im.copy()

	if len(contours)>0:

		
		cnt = contours[-1]



		#Contour Moments
		M = cv2.moments(cnt)
		#Contour Centroid
		#cx = int(M['m10']/M['m00'])
		#cy = int(M['m01']/M['m00'])
		#Contour Area
		area = cv2.contourArea(cnt)
		#Contour Perimeter
		perimeter = cv2.arcLength(cnt,True)
		# convert image back to BGR
		#color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
	

		if area>=25000  :
			# draw bounding rect contours onto image
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.int0(box)

			
			cv2.drawContours(overlay,[box],0,(0,0,255),2)
			x1,y1,w1,h1 = cv2.boundingRect(cnt)
			cv2.rectangle(overlay,(x1,y1),(x1+w1,y1+h1),(0,255,0),2)
			cv2.line(overlay, (int(x1+(w1/2))-10, int(y1+(h1/2))), (int(x1+(w1/2))+10, int(y1+(h1/2))), (0,255,0), 3)
			cv2.line(overlay, (int(x1+(w1/2)), int(y1+(h1/2))-10), (int(x1+(w1/2)), int(y1+(h1/2))+10), (0,255,0), 3)	
			cv2.putText(overlay,"Area={}".format(int(area)),(10,360), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),1)
			cv2.putText(overlay,"Perimeter={}".format(int(perimeter)),(10,320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),1)
			cv2.putText(overlay,"Lingkaran Kecil={}".format(lingkaran),(10,300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),1)
			cv2.rectangle(overlay, (x, y), (x+w, y+h), (255,0, 0), 2)	
			keypoints1 = detector1.detect(thresh)

			for k1 in keypoints1 :
				font1 = cv2.FONT_HERSHEY_SIMPLEX
				titikx11 = int(x1)
				titiky11 = int(y1)
				titikx21 = int(x1+w1)
				titiky21 = int(y1+h1)

				#im_with_keypoints = cv2.drawKeypoints(thresh, keypoints1, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

				#For Drawing Circle, Lines, and Number on Detected Blobs
				cv2.circle(overlay, ((int(k1.pt[0])), int(k1.pt[1])), int((k1.size)/2), (255, 0, 255), -1)
				cv2.line(overlay, (int(x1+(w1/2))-10, int(y1+(h1/2))), (int(x1+(w1/2))+10, int(y1+(h1/2))), (0,255,0), 3)
				cv2.line(overlay, (int(x1+(w1/2)), int(y1+(h1/2))-10), (int(x1+(w1/2)), int(y1+(h1/2))+10), (0,255,0), 3)
				cv2.rectangle(overlay,(titikx11,titiky11), (titikx21,titiky21),(0,255, 0), 2)

			lingkaran= (len(keypoints1))	



			if (int(lingkaran))>=4 :

				cv2.putText(overlay,"OK",(x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)
				okflag = True
				ngflag = False
			if (int(lingkaran))<4 :
				cv2.putText(overlay,"NG",(x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)
				okflag = False
				ngflag = True



		if area<25000 and area>100:
			# draw bounding rect contours onto image
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.int0(box)

			
			cv2.drawContours(overlay,[box],0,(0,0,255),2)
			x1,y1,w1,h1 = cv2.boundingRect(cnt)
			cv2.rectangle(overlay,(x1,y1),(x1+w1,y1+h1),(0,255,0),2)
			cv2.line(overlay, (int(x1+(w1/2))-10, int(y1+(h1/2))), (int(x1+(w1/2))+10, int(y1+(h1/2))), (0,255,0), 3)
			cv2.line(overlay, (int(x1+(w1/2)), int(y1+(h1/2))-10), (int(x1+(w1/2)), int(y1+(h1/2))+10), (0,255,0), 3)	
			cv2.putText(overlay,"Area={}".format(int(area)),(10,360), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),1)
			cv2.putText(overlay,"Perimeter={}".format(int(perimeter)),(10,320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),1)
			cv2.putText(overlay,"Lingkaran Kecil={}".format(lingkaran),(10,300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),1)
			cv2.rectangle(overlay, (x, y), (x+w, y+h), (255,0, 0), 2)			
			cv2.putText(overlay,"NG",(x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)
			okflag = False
			ngflag = True


		if (int(x1+(w1/2)<=340 and int(x1+(w1/2))>=320)) and okflag== True:
			
			start = time.monotonic()
			statusok = True

			cv2.line(overlay,(340,0),(340,480),(255,200,0),10)
			okflag = True
			ngflag = False
		
		else :
			statusok = False		

		if (int(x1+(w1/2)<=200 and int(x1+(w1/2))>=180)) and okflag== True:
			end = time.monotonic()
			elapsed = end-start
			speed = (140*1.05)/elapsed
			
			



				
		if statusok != statusnow:
			if statusok == True:

				hitung+=1
				ok+=1
				#koorY = str(int(k.pt[1])-200)
				#arduino.write(bytearray(koorY+"\n","UTF-8"))
			statusnow = statusok
			
					
					

		if (int(x1+(w1/2)<=340 and int(x1+(w1/2))>=320)) and ngflag== True:
			start = time.monotonic()
			statusng = True

			cv2.line(overlay,(340,0),(340,480),(255,255,0),5)

			ngflag = False
			okflag = False



		else :
			statusng = False

		if (int(x1+(w1/2)<=200 and int(x1+(w1/2))>=180)) and ngflag== True:
			end = time.monotonic()
			elapsed = end-start
			speed = (140*1.05)/elapsed
							
		if statusng != statusnowng:
			if statusng == True:

				hitung+=1
				ng+=1
				
			statusnowng = statusng
			

	else :
		cv2.putText(overlay,"Contour not found",(10,350), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),1)

	im = cv2.drawContours(overlay, contours, -1, (0, 255, 0), 1)

	#cv2.imshow("contours", im)
	cv2.line(overlay,(340,0),(340,480),(0,255,255),5)
	cv2.putText(overlay,"Jumlah{}".format(hitung),(10,400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),1)
	cv2.putText(overlay,"Waktu:{}".format(int(elapsed)),(10,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),1)
	cv2.putText(overlay,"Kecepatan:{}mm/detik".format(int(speed)),(10,120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),1)
	cv2.putText(overlay,"OK={}".format(ok),(10,430), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),1)
	cv2.putText(overlay,"NG={}".format(ng),(10,470), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),1)

	opacity = 0.7
	cv2.addWeighted(overlay, opacity, im, 1 - opacity, 0, im)
	cv2.imshow("Output", im)
	# Uncomment to resize to fit output window if needed
	#im = cv2.resize(im, None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
	cv2.imshow('Thresh', thresh)
	

	k = cv2.waitKey(10) & 0xff
	if k == 27:
		
		break

camera.release()
cv2.destroyAllWindows()
