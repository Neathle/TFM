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
#Script to help in the initial selection of markers. The images received in the topic are proceseed and if rectangles with good characteristics
#are found they are published in the output image marked with a red contour. 

class detector:
    def __init__(self):
        #1. ROS configuration
        self.bridge = CvBridge()
        self.pub_marker     = rospy.Publisher('detected_markers', messagedet, queue_size=1)
        self.pub_projection = rospy.Publisher('detector_img', Image, queue_size=1)
        self.pub_num_marker = rospy.Publisher('num_detecte_markers', num_markers, queue_size=10)
       
        #2. Parameters SETUP
            #Contour detection parameters
        self.thresh_binary  = cv2.THRESH_BINARY#THRESH_BINARY_INV busca objetos oscuros con fondo claro, THRESH_BINARY busca objetos claros con fondo oscuro
            #Rectangle detection parameters minimun width and height for images to loook
        self.min_w_h_image = 10 
        
        #3. START detector process when img is received in topic
        topic = rospy.get_param('/detectorSIFT_selectMarkers/img_topic')
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        
         #4. INICIALIZAR SIFT to find min set of kp and des 
        #self.SIFTdetector= cv2.ORB_create(nfeatures=100000, scoreType=cv2.ORB_HARRIS_SCORE)
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
                    #calcular SIFT y si tiene al menos kp>0, des>25, y de tamañao al menos 50x50 entonces exportar
                    kp, des = self.SIFTdetector.detectAndCompute(rectangle_cropped, None)
                    print("para el rectangulo detectado se encontraron kp",len(kp), " y des", len(des))
                    if len(kp)>0 and len(des)>10:
                        print ("Good rectangle found w/enough key points")
                        path = "/home/fer/catkin_ws/src/amcl_hybrid/marker_map_creator/0_select_markers/"
                        timedate = time.strftime("%Y%m%d-%H%M%S") 
                        rectangle_export = cv2.imwrite(path+'square_det_'+str(timedate)+'.bmp', rectangle_cropped)       
                        file = open(path+'square_det_'+str(timedate)+'.txt', 'w') 
                        file.write(str(data.header.seq))
                        file.write('\n') 
                        file.write(str(data.header.stamp))
                        file.write('\n') 
                        file.write(str(rectangle_crop_points))
                        file.write('\n')
                        file.close()
                else:
                    #no good rectangles were found
                    print( ".")
                    det_img = cv_image
                    
            #4. MOSTRAR Y PUBLICAR IMGS Y DATOS DEL PROCESO           
            contours_concat = np.concatenate((cv_image,   contours_img),   axis=1) 
            visualizar      = np.concatenate((contours_concat, rectangles_img), axis=1)
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
        return found_rectangle, rectangles_img, rectangle_crop_img, rectangle_found_points


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