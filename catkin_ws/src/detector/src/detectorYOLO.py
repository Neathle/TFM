#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import os
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from detector.msg import messagedet, num_markers
from detector.msg import marker as msg_marker
from geometry_msgs.msg import Point32
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
from detector.msg import BoundingBoxes


class detector:
    def __init__(self):
        #ROS configuration
        self.bridge = CvBridge()
        self.pub_marker = rospy.Publisher('detected_markers', messagedet, queue_size=1)
        self.pub_projection = rospy.Publisher('detector_img', Image, queue_size=1)
        self.pub_num_marker = rospy.Publisher('num_detecte_markers', num_markers, queue_size=10)

        #General detector parameters
        self.corner_colors = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)) ## BGR coners
        
        self.export_img_det = True
        self.export_path    = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
        
        #Contour detection parameters
        self.thresh_binary  = cv2.THRESH_BINARY_INV #THRESH_BINARY_INV busca objetos oscuros con fondo claro, THRESH_BINARY busca objetos claros con fondo oscuro
            #Rectangle detection parameters minimun width and height for images to loook
        self.min_w_h_image = 10 
        
        #3. INICIALIZAR SIFT
        #self.SIFTdetector = cv2.xfeatures2d.SIFT_create()
        
        self.SIFTdetector = cv2.ORB_create()
        #Min number of features matching between analyzed rectangle and stored marker to be consider as a detection
        self.minNumMatches = 7
        
        #CREAR LISTA DE MARCADORES A BUSCAR
        self.markers = (list(), list(), list()) #se crea lista de 3D para almacenar img, clase y ID de cada uno de los marcadores 
        self.numMarkers = 0
        
        folder = "/home/fer/catkin_ws/src/amcl_hybrid/detector/markers_alma/" #PENDING take from launch file
        files = os.listdir(folder)
        for file in files:
            if "marker" and "class" in file:
                path = folder + file
                #print "path is: ", path
                img = cv2.imread(path, 0)
                self.markers[0].append(img)
                marker_class = file.split("class",1)[1]
                marker_class = marker_class.split(".png",1)[0]
                self.markers[1].append(marker_class)
                marker_id = file.split("_",1)[0]
                marker_id = marker_id.split("marker",1)[1]
                self.markers[2].append(int(marker_id))
                self.numMarkers = self.numMarkers + 1
                #print("total images in folder: " + str(self.numMarkers))
        
        #ROS declare topics, start detector process when img is received in topic
        yolo_msg_topic  = rospy.get_param('/detectorYOLO/yolo_msg_topic')
        self.yolo_msg_sub = rospy.Subscriber(yolo_msg_topic, BoundingBoxes, self.yolo_msg_callback)
        yolo_img_topic  = rospy.get_param('/detectorYOLO/yolo_img_topic')
        self.image_sub = rospy.Subscriber(yolo_img_topic, Image, self.rgb_img_callback)
        
        #Inicializar variables de buffer y yolo flag
        self.yolo_detect_flag = False
        self.current_img = (list(), list())


    def yolo_msg_callback(self, yolo_msg):
        try:
            #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #print "el msj yolo recibido es: ", yolo_msg

            #CREAR OBJETO GLOBAL PARA ALMACENAR DATOS YOLO
            self.yolo_detections = (list(), list(), list(), list(), list(), list()) #0. class label, 1. probability, 2. xmin, ymin, xmax, ymax, 3. id
            
            #self.yolo_detections: 0. class label, 1. probability, 2. xmin, ymin, xmax, ymax, 3. id
            for msg in yolo_msg.bounding_boxes:
                #print "!"
                if msg.probability > 0.7: #prob detection bigger than 70%
                    #print "70!!"
                    for j in range(0,len(self.markers[1])):
                        #print "%"
                        if msg.Class == self.markers[1][j]: #if the class of a detected objetc matches a marker class then we store yolo info
                            #print"class detected!!!!"
                            #print "yolo rcvd with >80% and class is ", msg.Class
                            bounding_box_points = (msg.xmin, msg.xmax, msg.ymin, msg.ymax)
                            self.yolo_detections[0].append(msg.Class)
                            self.yolo_detections[1].append(msg.probability)
                            self.yolo_detections[2].append(bounding_box_points)
                            self.yolo_detections[3].append(yolo_msg.image_header.stamp)                           
                            self.yolo_detections[4].append(self.markers[2][j]) #store the corresponding marker ID
                            self.yolo_detections[5].append(self.markers[0][j]) #store the correspoinding marker img
                            #print "the stored img tstamp is ", self.yolo_detections[3][0]
                            #print "the detected object is", self.yolo_detections[0][0], "with a probability of", self.yolo_detections[1][0], "the marker ID is", self.yolo_detections[4][0]
                            self.yolo_detect_flag = True
        except:
            print("yolo callback error")
            
    def rgb_img_callback(self, img):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
            #print "el current tstamp del RGB img y secuencia es", img.header.stamp, img.header.seq
            #store incoming imgs from robot in buffer to compensate delay between yolo and detector 
            self.current_img[0].append(cv_img)
            self.current_img[1].append(img.header.stamp)
            
            if self.yolo_detect_flag == True:    
                #print "YOLO flag is on babyyyyyy!!"
                #print "the len of yolo detections in current img is", len(self.yolo_detections[0]), self.yolo_detections[0]
                for i in range(0,len(self.yolo_detections[0])):  #loop over all the objects detected by yolo network in a single image
                    #exportar datos de la marca detectada
                    timedate = time.strftime("%Y%m%d-%H%M%S") 
                    path = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
                    file = open(path+'yolo_det'+str(timedate)+'.txt', 'w') 
                     
                    for k in range(0,len(self.current_img[0])): #loop over all imgs in buffer
                        if self.current_img[1][k] == self.yolo_detections[3][i]: #if yolo img tstamp matches img tstamp in buffer 
                            current_img_index = k
                            detection_img = self.current_img[0][k]
                            xmin = self.yolo_detections[2][i][0]
                            ymin = self.yolo_detections[2][i][1]
                            xmax = self.yolo_detections[2][i][2]
                            ymax = self.yolo_detections[2][i][3]
                            detection_img_crop = detection_img[ymin:ymax, xmin:xmax] #cut rgb image with yolo detected bounding box corners
                            
                            detection_img_crop_gray = cv2.cvtColor(detection_img_crop, cv2.COLOR_BGR2GRAY) #convertir a escala de grises para dsps obtener height y width con .shape
                            det_crop_w, det_crop_h = detection_img_crop_gray.shape
                            #print "!!!!!!!!!"
                            #print "the shape of the bbox img is width", det_crop_w, "height", det_crop_h
                            #marker_gray = cv2.cvtColor(self.yolo_detections[5][i], cv2.COLOR_BGR2GRAY)
                            marker_img_w, marker_img_h = self.yolo_detections[5][i].shape[:2] 
                            #print "the shape of the marker image is width", marker_img_w, "height", marker_img_h
                            #print "!!!!!!!!!"
                            if (det_crop_w < marker_img_w/2) or (det_crop_h < marker_img_h/2):  #si bbox crop ims es menor a 0.5marker desechar, no queremos esquinas o pedacitos del marcador
                                #print "error, bounding box is smaller than marker, not useful"
                                res_export  = cv2.imwrite(path+'det_bbox_crop_BAD_'+str(timedate)+'.bmp',detection_img_crop) 
                            
                            else:
                                
                                res_export  = cv2.imwrite(path+'det_bbox_crop_GOOD_'+str(timedate)+'.bmp',detection_img_crop) 
                                #FIND rectangle in img 
                                self.thresh_binary  = cv2.THRESH_BINARY_INV 
                                contours, contours_img, threshold_img = self.findContours(detection_img_crop)
                                #Use both types of binarization
                                self.thresh_binary  = cv2.THRESH_BINARY
                                contours2, contours_img2, threshold_img2 = self.findContours(detection_img_crop)
                                
                                contours_bin_bininv = contours + contours2                               
                                
                                
                                #PENDING variar un poco las condiciones de los rectangulos
                                for cnt in contours_bin_bininv:
                                        found_rectangle, rectangles_img, rectangle_cropped, rectangle_crop_points = self.findRectangles(cnt, detection_img_crop)
                                        
                                        #PENDING publish message mas corto??
                                        #3. Compare found rectangle to markers stored in self.markers
                                        if found_rectangle == True:
                                            
                                            #print "corners found for object are ", rectangle_crop_points
                                            #markerID = self.compareImage(rectangle_cropped) 
                                            markerID =  self.yolo_detections[4][i]
                                            corners, det_img = self.sortCorners(detection_img_crop, rectangle_crop_points, markerID)
                                            #5. Publish Marker Msg in TOPIC
                                            msg_marker = messagedet() 
                                            msg_marker.DetectedMarkers.append(self.makeMsgMarker(markerID, corners))   
                                            msg_marker.header.seq = rospy.Duration()
                                            msg_marker.header.stamp = rospy.Time.now()
                                            frame_id = rospy.get_param('/detectorYOLO/detection_msg_publish_topic')
                                            msg_marker.header.frame_id = frame_id
                                            self.pub_marker.publish(msg_marker)
                                            
                                            n_markers = num_markers()
                                            n_markers.header.stamp = rospy.Time.now()
                                            n_markers.number = UInt8(len(msg_marker.DetectedMarkers))
                                            self.pub_num_marker.publish(n_markers)
                                            #print("publico -> " + str(len(msg_marker.DetectedMarkers)))
                                            #print(msg_marker) 
                                        else:
                                            print ("no good rectangles were found")
                          
                                rectangle_export  = cv2.imwrite(path+'yolo_det_rectangles_PRUEBA.bmp',rectangles_img)    
                                #print "se encontro y exporto un rectangulo de bbox croped"
                                
                                contours_concat = np.concatenate((threshold_img,   contours_img),   axis=1) 
                                detector_img    = np.concatenate((contours_concat, rectangles_img), axis=1)
                                #print "detector_img type is ", type(detector_img)
                                result_export  = cv2.imwrite(path+'yolo_det_rectangles_process_'+str(timedate)+'.bmp',detector_img) 
                                
                            #calculate delay between rgb data transmision and yolo detection
                            #PENDING hacer una funcion de todo esto abajo
                            #delay = img.header.stamp - self.yolo_detections[3][i]
                            ##print "the delay is", delay.secs, "segs", delay.nsecs, "nanoseg"
                            #file = open(path+'yolo_det'+str(timedate)+'.txt', 'w') 
                            #file.write(str(delay.secs))
                            #file.write(".")
                            #file.write(str(delay.nsecs))
                            #file.write('\n')
                            #file.close() #close debugging txt file
                                     
                self.yolo_detect_flag == False
                #PENDING clean image buffer
                #self.current_img[0] = self.current_img[0][current_img_index:-1]
                #self.current_img[1] = self.current_img[1][current_img_index:-1]
            
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
                ##print "Good rectangle found"
                #path = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
                #rectangle_export = cv2.imwrite(path+'square_det_'+str(timedate)+'.bmp', rectangle_crop_img)                    
        return found_rectangle, rectangles_img, rectangle_crop_img, rectangle_found_points
            
    def compareImage(self, yolo_img, marker):
        
        marker    = cv2.cvtColor(marker, cv2.COLOR_BGR2GRAY)
        kp, des = self.SIFTdetector.detectAndCompute(marker, None)
        
        yolo_img_original = yolo_img
        yolo_img = cv2.cvtColor(yolo_img, cv2.COLOR_BGR2GRAY)
        kp2, des2 = self.SIFTdetector.detectAndCompute(yolo_img, None)
        #print "hola1"       
        #print "los kp y des para la img yolo son: ", len(kp2), len(des2)
        
        if len(kp2) > 0 and len(des2)>20: #There must be at least 1 feature to compare and des>20 helps avoiding this error https://stackoverflow.com/questions/25089393/opencv-flannbasedmatcher
            #print "hola2"
            minNumMatches_found = list()
            #1. Compute flann matches between yolo_img and markers
            FLANN_INDEX_KDTREE = 1 
            index_params  = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
            search_params = dict(checks = 50)
            flann = cv2.FlannBasedMatcher(index_params, search_params)
            matches_found = flann.knnMatch(des,des2,k=2) 
            #print "hola3"
            #2. Store good matches that pass the Lowe's ratio test.
            good_matches = []
            for m,n in matches_found:
                if m.distance < 0.95*n.distance:
                    good_matches.append(m)
            #print("Filtered num of matches per Lowe's tests is", len(good_matches))
            #print "min num of matches required is: ", self.minNumMatches
            #print "hola4"
            #3. If the number of good matches found is >= minNumMatches, then use homography matrix and RANSAC algorithm to detect the marker in the rectangle image
            if len(good_matches)>=self.minNumMatches:
                src_pts = np.float32([  kp[m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
                dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)
                #print "hola5"
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                matchesMask = mask.ravel().tolist()
            
                h,w = marker.shape
                pts = np.float32([ [0,0],[0,h],[w,h],[w,0] ]).reshape(-1,1,2)
                dst = cv2.perspectiveTransform(pts,M)
            
                #Draw detected marker 
                #marker_detected_by_homography = cv2.polylines(original_img,[np.int32(dst)],True,(0,255,0),3, cv2.LINE_AA)
                marker_detected_by_homography = cv2.polylines(yolo_img_original,[np.int32(dst)],True,(255,0,0), 3) 
                
                draw_params = dict(matchColor = (0,255,0), singlePointColor = None, matchesMask = matchesMask, flags = 2)
                img3 = cv2.drawMatches(marker,kp,yolo_img,kp2,good_matches,None,**draw_params)
                #print "hola6"                 
                #Exportar image to folder
                if self.export_img_det == True:
                    sift_export = cv2.imwrite(self.export_path+'det_yolo_sift_kp'+str(time.strftime("%Y%m%d-%H%M%S"))+'.bmp', img3) 
                    #print "hola7"
                #Function Output
                return marker_detected_by_homography, dst
            else:
                print ("     Not enough matches are found - %d/%d" % (len(good_matches),self.minNumMatches) )
                #print "hola8"
        else:            
            print ("(Error condition: (kp2) > 0 and len(des2)>25 not met")
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
                #print("Error of Cuadrant or a corner equal to centroid point-irregular polygons is best to avoid")
                return 0
        
        text = str("ID:" + str(markerID))
        corners_img = cv2.putText(img, text, (w_center, h_center), 2, 1, (255, 0, 0), 1, lineType=cv2.LINE_AA)
        corners_img = cv2.drawContours(img, [corners], -1, (0, 255, 255), 1, cv2.LINE_AA) #draw rectangles to be debuged
        timedate = time.strftime("%Y%m%d-%H%M%S") 
        path = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
        rectangle_export = cv2.imwrite(path+'corners_img_'+str(timedate)+'.bmp',corners_img)
        
        return sorted_corner, corners_img

    def makeMsgMarker(self, numMarkers, corners):
        new_marker = msg_marker()
        #new_marker = [corners, 0, 0, numMarkers]
        for corner in corners:
            pixel = Point32()
            pixel.x = corner[0]
            pixel.y = corner[1]
            pixel.z = 0
            ##print(pixel)
            new_marker.Corners.append(pixel)
        else:
            new_marker.map = UInt8(0)
            new_marker.sector = UInt8(0)
            new_marker.ID = UInt8(numMarkers)
            ##print(new_marker)
            return new_marker
        #print("Error make marker msg")

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