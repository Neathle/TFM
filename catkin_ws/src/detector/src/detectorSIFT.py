#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from detector.msg import messagedet, num_markers
from detector.msg import marker as msg_marker
from geometry_msgs.msg import Point32
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
import os
import time


class detector:
    def __init__(self):
        #1. ROS configuration
        self.bridge = CvBridge()
        self.pub_marker           = rospy.Publisher('detected_markers', messagedet, queue_size=1)
        self.pub_projection      = rospy.Publisher('detector_img', Image, queue_size=1)
        self.pub_num_marker = rospy.Publisher('num_detecte_markers', num_markers, queue_size=10)
       
        #2. Parameters SETUP
        markers_path     = rospy.get_param('/detectorSIFT/markers_path')
        self.export_path = rospy.get_param('/detectorSIFT/export_path')
        self.thresh_bin   = rospy.get_param('/detectorSIFT/thresh_bin')
        self.corner_colors  = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)) #BGR coners
        self.export_img_det = True
        
        lst = os.listdir(markers_path)
        self.numMarkers  = len(lst)
        #Contour detection parameters
        if self.thresh_bin == "bin":
            self.thresh_binary  = cv2.THRESH_BINARY #THRESH_BINARY busca objetos claros con fondo oscuro
        else:
            self.thresh_binary  = cv2.THRESH_BINARY_INV #THRESH_BINARY_INV busca objetos oscuros con fondo claro
        print("threshold binary type set is ", self.thresh_bin)
            
        #Rectangle detection parameters minimun width and height for images to loook
        self.min_w_h_image = 10 
        #Min number of features matching between analyzed rectangle and stored marker to be consider as a detection
        self.minNumMatches = 20
        
        #3. INICIALIZAR SIFT
        self.SIFTdetector=  cv2.ORB_create(
                                                                        nfeatures = 500,       #max num of features to retain.
                                                                        scaleFactor = 1.2,    #Pyramid decimation ratio, greater than 1
                                                                        nlevels = 8,               #The number of pyramid levels.
                                                                        edgeThreshold = 7,#This is size of the border where the features are not detected. It should roughly match the patchSize parameter
                                                                        firstLevel = 0,           # It should be 0 in the current implementation.
                                                                        WTA_K = 2,             #The number of points that produce each element of the oriented BRIEF descriptor.
                                                                        scoreType = cv2.ORB_HARRIS_SCORE,   # The default HARRIS_SCORE means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is 
                                                                                                          # used to retain best nfeatures features); FAST_SCORE is alternative value of the parameter that produces slightly less stable kps but a little faster to compute.   #scoreType = cv2.ORB_FAST_SCORE,
                                                                        patchSize = 7          # size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.
                                                                        )                                 #https://stackoverflow.com/questions/65602353/how-to-use-orb-feature-detector-with-small-images-in-opencv
        #self.SIFTdetector = cv2.xfeatures2d.SURF_create(self.numMarkers)
        #self.SIFTdetector = cv2.xfeatures2d.SIFT_create() SIFT not available in this cv version, must use ORB which is opensource
        
        #4. CREAR LISTA DE MARCADORES A BUSCAR
        self.markers = (list(), list(), list()) #se crea lista3D para almacenar img, keypoints, descriptors de cada uno de los marcadores en folder
        for i in range(0, self.numMarkers):
            path = str(markers_path + "marker" + str(i) +".bmp") 
            img = cv2.imread(path, 0)
            self.markers[0].append(img)
            kp, des = self.SIFTdetector.detectAndCompute(img, None)
            self.markers[1].append(kp)
            self.markers[2].append(des)   
            print("path is ", path)
            print("i es", i, "self.numMarkers es ", self.numMarkers)
            print("para el marker",i," se encontraron kp",len(kp), " y des", len(des))
        print("total images in folder: " + str(len(self.markers[0])))
        
        #5. START detector process when img is received in topic
        topic = rospy.get_param('/detectorSIFT/img_topic')
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            canvas = cv_image.copy()
            
            #1. Find contours in the image received from topic
            contours, contours_img, threshold_img = self.findContours(canvas)
            
            #2. Find rectangles looping over all the contours
            for cnt in contours:
                found_rectangle, rectangles_img, rectangle_cropped, rectangle_crop_points = self.findRectangles(cnt, canvas)
                
                #3. Compare found rectangle to markers stored in self.markers
                if found_rectangle == True:
                    markerID = self.compareImage(rectangle_cropped) 
                    #markerID = self.compareImage(cv_image)
                    
                    #4. If marker was found sort corners
                    if  markerID >= 0: 
                    #if  markerID is not None: 
                        corners, det_img = self.sortCorners(cv_image, rectangle_crop_points, markerID)
                        #5. Publish Marker Msg in TOPIC
                        msg_marker = messagedet() 
                        msg_marker.DetectedMarkers.append(self.makeMsgMarker(markerID, corners))   
                        msg_marker.header.seq = rospy.Duration()
                        msg_marker.header.stamp = rospy.Time.now()
                        frame_id = rospy.get_param('/detectorSIFT/detection_msg_publish_topic')
                        msg_marker.header.frame_id = frame_id
                        self.pub_marker.publish(msg_marker)
                        
                        n_markers = num_markers()
                        n_markers.header.stamp = rospy.Time.now()
                        n_markers.number = UInt8(len(msg_marker.DetectedMarkers))
                        self.pub_num_marker.publish(n_markers)
                        print("publico -> " + str(len(msg_marker.DetectedMarkers)))
                        print(msg_marker) 
                    else:
                        #a good rectangle was found but no marker detected
                        det_img = cv_image
                else:
                    #no good rectangles were found
                    det_img = cv_image
                    
            ##6. MOSTRAR Y PUBLICAR IMGS DEL PROCESO
            #cv2.rectangle(cuadros, (5,5),(self.original_width-5,self.original_height-5),(0,0,255),1) #cuadro de Miguel para dejar orilla de 5pixeles
            #cv2.rectangle(cuadros, (5,5),(15,15),(255,0,0),1) #medida minima para buscar marcas, cuadro de 10x10pixels
            
            contours_concat = np.concatenate((threshold_img,   contours_img),   axis=1) 
            contour_rect    = np.concatenate((contours_concat, rectangles_img), axis=1)
            visualizar      = np.concatenate((contour_rect,    det_img),        axis=1)
            detector_img    = self.bridge.cv2_to_imgmsg(visualizar, encoding="bgr8")
            self.pub_projection.publish(detector_img)
            
        except CvBridgeError as e:
            print(e)
        
    def CrossProduct(self, A):
        X1 = (A[1][0] - A[0][0]) # Stores coefficient of X # direction of vector A[1]A[0]
        Y1 = (A[1][1] - A[0][1]) # Stores coefficient of Y # direction of vector A[1]A[0]
        X2 = (A[2][0] - A[0][0]) # Stores coefficient of X # direction of vector A[2]A[0]
        Y2 = (A[2][1] - A[0][1]) # Stores coefficient of Y # direction of vector A[2]A[0] 
        return (X1 * Y2 - Y1 * X2) # Return cross product
 
    def isConvex(self, points): # Function to check if the polygon is convex or not https://www.geeksforgeeks.org/check-if-given-polygon-is-a-convex-polygon-or-not/
        N = len(points) # Stores i of edges in polygon
        prev = 0 # Stores direction of cross product of previous traversed edges
        curr = 0 # Stores direction of cross product of current traversed edges
        for i in range(N):     # Traverse the array
            temp = [points[i], points[(i + 1) % N],
                            points[(i + 2) % N]] # Stores three adjacent edges of the polygon
            curr = self.CrossProduct(temp)  # Update curr
            if (curr != 0):  # If curr is not equal to 0
                if (curr * prev < 0):  # If direction of cross product of all adjacent edges are not same
                    return False
                else:
                    prev = curr   # Update curr
        return True

    def findContours(self, img):
        #1. Change color space and extract gray channel
        blurred_img = cv2.GaussianBlur(img, (5,5), 0)  
        hsv = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV) #convert to hsv color space
        h,s,v = cv2.split(hsv) #split the channels
        
        #2. Apply umbralization 
        th, threshed = cv2.threshold(v, 0, 255, self.thresh_binary | cv2.THRESH_OTSU) #Use Otsu automatic thresholding value 
        
        #3. Find Contours in the umbralized image
        contours = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]## contours is a list with len = contours number, find contours using threshold
        
        #Export images of color space change, grayscale and thresholding 
        v_bgr = cv2.cvtColor(v, cv2.COLOR_GRAY2BGR) #convert thresh img to 3 channels to concatenate
        v_bgr = np.concatenate((hsv, v_bgr), axis=1)
        threshed_bgr = cv2.cvtColor(threshed, cv2.COLOR_GRAY2BGR) #convert thresh img to 3 channels to concatenate
        threshold_img = np.concatenate((v_bgr, threshed_bgr), axis=1)
        #Export image of contour detection
        contours_img  = img.copy()
        contours_img = cv2.drawContours(contours_img, contours, -1, (0,255,0), 1)

        return contours, contours_img, threshold_img

    def findRectangles(self, contour, original_img):
        timedate = time.strftime("%Y%m%d-%H%M%S") 
        #inicializar las variables a cero o valores redundantes para que si no se encuentran rectangulos evitar errores con el retorno de la funcion
        found_rectangle = False  
        rectangle_crop_img = np.zeros((100,100,3), dtype=np.uint8) #return empty img
        rectangle_found_points = ([0, 0], [0, 0], [0, 0], [0, 0])
        rectangles_img = original_img.copy()
             
        arclen = cv2.arcLength(contour, True)
        rectangle_points = cv2.approxPolyDP(contour, 0.02* arclen, True)  ###IMPORTANTE! esta es la funcion que encuentra los rectangulos/poligonos
        
        if len(rectangle_points) == 4: ## solo entramos si es un rectangulo, four corners
            x1 ,y1, w, h = cv2.boundingRect(rectangle_points)
            rectangles_img = cv2.drawContours(original_img, [rectangle_points], -1, (255, 0, 0), 1, cv2.LINE_AA) #draw all rectangles found in blue
            #hacemos calculos para recortar
            w_img_ = list()
            h_img_ = list()
            for point in rectangle_points:
                w_img_.append(point[0][0])
                h_img_.append(point[0][1])

            w_img = sorted(w_img_)
            h_img = sorted(h_img_)
            w_img_max = w_img[0]+w
            h_img_max = h_img[0]+h
            #Las sig. 4 lineas son para adaptar el formato de la lista de puntos a la funcion isconvex
            approx_list = np.ndarray.tolist(rectangle_points)
            convex_list = []
            for i in range(0,4):
                convex_list.append(approx_list[i][0])
                
            #CONDICIONES PARA FILTRAR CUADROS tam minimo y poligono convexo
            if w >= self.min_w_h_image  and  h >= self.min_w_h_image and self.isConvex(convex_list):
                rectangle_crop_img = original_img[h_img[0]:h_img_max, w_img[0]:w_img_max] #Recortamos la imagen del contorno   
                rectangles_img = cv2.drawContours(rectangles_img, [rectangle_points], -1, (0, 0, 255), 1, cv2.LINE_AA) #draw good rectangles in red
                found_rectangle = True
                rectangle_found_points = rectangle_points
                #print "Good rectangle found"
                #path = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
                #rectangle_export = cv2.imwrite(path+'square_det_'+str(timedate)+'.bmp', rectangle_crop_img)                    
        return found_rectangle, rectangles_img, rectangle_crop_img, rectangle_found_points

    def compareImage(self, rectangle):
        rectangle_original = rectangle
        rectangle = cv2.cvtColor(rectangle, cv2.COLOR_BGR2GRAY)
        kp, des = self.SIFTdetector.detectAndCompute(rectangle, None)

        #if len(kp) > 0 and len(des)>25: #len(des)>25 #There must be at least 1 feature to compare and des>20 helps avoiding this error https://stackoverflow.com/questions/25089393/opencv-flannbasedmatcher
        if len(kp) > 30 and len(des)>60: #len(des)>25 #There must be at least 1 feature to compare and des>20 helps avoiding this error https://stackoverflow.com/questions/25089393/opencv-flannbasedmatcher
            minNumMatches_found = list()
            for i in range(self.numMarkers): #compare rectangle to all the markers stored, logic from:https://docs.opencv.org/3.4/d1/de0/tutorial_py_feature_homography.html         
                #1. Compute flann matches between rectangle and markers
                FLANN_INDEX_KDTREE = 1 
                index_params  = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
                search_params = dict(checks = 50)
                flann = cv2.FlannBasedMatcher(index_params, search_params)
                ##des2 = des.convertTo(descriptors_1, CV_32F)
                ##des1 = self.markers[2][i].convertTo(descriptors_1, CV_32F)
                print("para el rectangulo actual se encontraron kp",len(kp), " y des", len(des))
                print ("Comparing good rect with enough features to marker", i)
                des2 = np.float32(des)
                des1 = np.float32(self.markers[2][i])
                
                matches_found = flann.knnMatch(des1,des2,k=2) #updated matcher from bf to flann in order to perform lowe's ratio test
                #matches_found = flann.knnMatch(self.markers[2][i],des,k=2) #updated matcher from bf to flann in order to perform lowe's ratio test
                print ("Num unfiltered matches found ", len(matches_found))
                
                #2. Store good matches that pass the Lowe's ratio test.
                good_matches = []
                for m,n in matches_found:
                    if m.distance < 0.6*n.distance:
                        good_matches.append(m)
                        markerID = i
                print("Filtered num of matches per Lowe's tests is", len(good_matches))
                
                #3. If the number of good matches found is >= minNumMatches, then use homography matrix and RANSAC algorithm to detect the marker in the rectangle image
                MIN_MATCH_COUNT = self.minNumMatches
                if len(good_matches)>MIN_MATCH_COUNT:
                    src_pts = np.float32([ self.markers[1][i][m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
                    dst_pts = np.float32([ kp[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)
                
                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                    matchesMask = mask.ravel().tolist()
                
                    h,w = rectangle.shape
                    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                    dst = cv2.perspectiveTransform(pts,M)
                
                    #Draw detected marker 
                    #marker_detected_by_homography = cv2.polylines(original_img,[np.int32(dst)],True,(0,255,0),3, cv2.LINE_AA)
                    marker_detected_by_homography = cv2.polylines(rectangle_original,[np.int32(dst)],True,(0,0,0),0, cv2.LINE_AA) 
                    
                    draw_params = dict(matchColor = (0,255,0), singlePointColor = None, matchesMask = matchesMask, flags = 2)
                    img3 = cv2.drawMatches(self.markers[0][i],self.markers[1][i],marker_detected_by_homography,kp,good_matches,None,**draw_params)
                                        
                    #Exportar image to folder
                    if self.export_img_det == True:
                        sift_export = cv2.imwrite(self.export_path+'detected_'+str(time.strftime("%Y%m%d-%H%M%S"))+'.bmp', img3) 
                    
                    #Function Output
                    print ("The marker found ID is", markerID)
                    return markerID
                else:
                    print ("     Not enough matches are found - %d/%d" % (len(good_matches),MIN_MATCH_COUNT))
        else:
            return -1
             
    def sortCorners(self, img, corners, markerID):
        sorted_corner = [list(), list(), list(), list()]
        #find rectangle centroid, (it can be an irregular rectangle so we want the centroid instead of the center) https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
        M = cv2.moments(corners)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        w_center = cx
        h_center = cy                 
               
        #clasify corners according to their position compared to the centroid point
        for corner in corners:
            if corner[0][0] < w_center and corner[0][1] < h_center:   #1er cuadrante, arriba izq, azul
                sorted_corner[0] = corner[0]
                corners_img = cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.corner_colors[0], -1) 
            elif corner[0][0] > w_center and corner[0][1] < h_center: #2do cuadrante, arriba der, verde
                sorted_corner[1] = corner[0]
                corners_img = cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.corner_colors[1], -1)
            elif corner[0][0] > w_center and corner[0][1] > h_center: #3er cuadrante, abajo der, rojo
                sorted_corner[2] = corner[0]
                corners_img = cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.corner_colors[2], -1)
            elif corner[0][0] < w_center and corner[0][1] > h_center: #4to cuadrante, abajo izq, amarillo
                sorted_corner[3] = corner[0]
                corners_img = cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.corner_colors[3], -1)
            else:
                print("Error of Cuadrant or a corner equal to centroid point-irregular polygons is best to avoid")
                return 0
        
        text = str("ID:" + str(markerID))
        corners_img = cv2.putText(img, text, (w_center-50, h_center), 2, 1, (255, 0, 0), 1, lineType=cv2.LINE_AA)
        corners_img = cv2.drawContours(img, [corners], -1, (0, 255, 255), 1, cv2.LINE_AA) #draw rectangles to be debuged
        timedate = time.strftime("%Y%m%d-%H%M%S") 
        path = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
        rectangle_export = cv2.imwrite(path+'corners_img_'+str(timedate)+'.bmp',corners_img)
        
        return sorted_corner, corners_img

    def makeMsgMarker(self, markerID, corners):
        #print("Make msg marker")
        #print(corners)
        new_marker = msg_marker()
        #new_marker = [corners, 0, 0, markerID]
        for corner in corners:
            pixel = Point32()
            #print(corner)
            pixel.x = corner[0]
            pixel.y = corner[1]
            pixel.z = 0
            #print(pixel)
            new_marker.Corners.append(pixel)
        else:
            new_marker.map = UInt8(0)
            new_marker.sector = UInt8(0)
            new_marker.ID = UInt8(markerID)
            #print(new_marker)
            return new_marker

def main(args):
    rospy.init_node('detector_SIFT', anonymous=True)
    detector()  
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)