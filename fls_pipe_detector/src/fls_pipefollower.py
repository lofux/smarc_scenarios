#!/usr/bin/env python

import rospy
import sys
import cv2
import cv2 as cv
import std_msgs.msg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np

class cvBridge():
    def __init__(self):

        self.node_name = "cv_bridge"

        rospy.init_node(self.node_name)

        ## Create the OpenCV display window for the RGB image
        #self.cv_window_name = self.node_name
        #cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_AUTOSIZE)
        #cv.MoveWindow(self.cv_window_name, 25, 75)

        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber("/lolo_auv/depth/image_raw_raw_sonar", Image, self.image_callback)

        # Publish landmark pose-data in 2D
        #self.pub = rospy.Publisher('/lolo_auv/sonar_frame_landmarks', Pose2D, queue_size=10)
        self.pub_array = rospy.Publisher('/lolo_auv/sonar_frame_landmarks_Array', PoseArray, queue_size=10)

        #Publish image with pipeLine detected and line drawn on top
        self.pub_pipe_img = rospy.Publisher('/lolo_auv/image_pipeline_detected', Image, queue_size=10)
        
        
        rospy.loginfo("Waiting for image topics...")

    def image_callback(self,ros_image):

         # Use cv_bridge() to convert the ROS image to OpenCV format
         try:
             #frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
             frame = self.bridge.imgmsg_to_cv2(ros_image, "32FC1")
             #frame.astype(np.uint8)
             #img = self.bridge.cv2_to_imgmsg(frame,"bgr8")
             
         except CvBridgeError, e:
             print e
        
         # Convert the image to a Numpy array since most cv2 functions
         # require Numpy arrays.

         frame = np.array(frame, dtype=np.float32)
         #frame = frame.astype(np.uint8) #svart bild...
         frame = (frame * 255).round().astype(np.uint8)
         #frame = np.array(frame, dtype=np.uint8)

         #test some different filters
         median = cv2.medianBlur(frame,7)  #default: median = cv2.medianBlur(frame,7)
         #blur = cv2.bilateralFilter(frame, 9, 75, 75) #ser bra ut!

         # dilate white areas
         kernel = np.ones((20, 20), np.uint8) #default kernel = np.ones((20, 20), np.uint8)
         dilation = cv2.dilate(median, kernel, iterations=1)
         opening = cv.morphologyEx(median, cv.MORPH_OPEN, kernel)
         # cv2.imshow(self.node_name, dilation)

         # Process the frame using the process_image() function
         display_image, lines = self.process_image(opening)
         angle, slopes = self.find_angles_from_lines(lines)
         #print lines, angle, slopes
         print len(lines)
         mat = self.dist_lines(lines,slopes)

         landmarks_x, landmarks_y, lineID = self.find_parLines(lines, mat)




         display_image2 = self.process_imageLandmarks(display_image,lines,mat)

         c = self.find_pipeLineLine(mat, lines, display_image)

         #print x
                       
         # Display the image.
         #cv2.imshow(self.node_name, display_image)

         #Publish the pose2d message with landmarks

         #lX = float(landmarks_x[1])
         #lY = float(landmarks_y[1])

         #msg = Pose2D(x=1., y=1., theta = 0.)
         #msg = Pose2D(x=lX, y=lY, theta=0)


         h = std_msgs.msg.Header()
         h.frame_id = "sonar_frame"
         h.stamp = rospy.Time.now()


         msgArray = PoseArray()
         msgArray.poses = []
         msgArray.header = h
         #msgArray.poses
         #landmark_pose = Pose()
         #landmark_point = Point()



         for i in range(1,len(landmarks_x)):
             landmark_pose = Pose()
             landmark_pose.position.x = float(landmarks_x[i])
             landmark_pose.position.y = float(landmarks_y[i])
             landmark_pose.position.z = 0.
             landmark_pose.orientation.x = 0.
             landmark_pose.orientation.y = 0.
             landmark_pose.orientation.z = 0.
             landmark_pose.orientation.w = 0.

             print "adding landmarks to poseArray!!! Pos...: "
             print landmark_pose

             msgArray.poses.append(landmark_pose) #only last value is kept! why?



         self.pub_array.publish(msgArray)

         #landmark_pose.position.x = 1
         #landmark_pose.position.y = 2
         #landmark_pose.position.z = 3

         #self.pub.publish(msg)


         #contours
         #ret, thresh = cv2.threshold(display_image, 127, 255, 0)
         #im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
         #cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
         #print contours
         
         #cv2.waitKey(0)
         cv2.destroyAllWindows()
         # Process any keyboard commands
         #self.keystroke = cv2.WaitKey(5)
         #if 32 <= self.keystroke and self.keystroke < 128:
         #    cc = chr(self.keystroke).lower()
         #    if cc == 'q':
         #        # The user has press the q key, so exit
         #        rospy.signal_shutdown("User hit q key to quit.")


    def process_image(self, frame):
         # Convert to greyscale
         #grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
         #grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
         grey = frame


         #grey.astype(np.uint8)
        
         #grey = cv2.blur(grey, (18, 18))
         grey = cv2.bilateralFilter(frame, 9, 75, 75)  # ser bra ut! #default grey = cv2.bilateralFilter(frame, 9, 75, 75)

         #grey = cv2.GaussianBlur(grey, (1,1),0)
         #kernel = np.ones((5, 5), np.float32) / 15
         #grey = cv2.filter2D(grey, -1, kernel)

         # Compute edges using the Canny edge filter
         edges = cv2.Canny(grey, 10.0, 80.0) #default: edges = cv2.Canny(grey, 10.0, 50.0)
         #edges =  frame
         img = grey


         #lines = cv2.HoughLinesP(edges,1,np.pi/180,15,minLineLength,maxLineGap)
         minVote = 10
         lines = cv2.HoughLinesP(edges,1,np.pi/180,minVote,10,40,20) #default lines = cv2.HoughLinesP(edges,1,np.pi/180,minVote,10,40,20)



 	 for x in range(0, len(lines)):
             for x1,y1,x2,y2 in lines[x]:
                 cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)

         #return edges

	 return img, lines

    def process_imageLandmarks(self,img,lines,mat):
        xLandmarks, yLandmarks, lineID = self.find_parLines(lines, mat)

        xLand = xLandmarks[0]
        yLand = yLandmarks[0]



        #cv2.circle(img,(xLand,yLand),5, (0,255,0))
        for i in range(1,len(xLandmarks)):
            cv2.circle(img,(xLandmarks[i], yLandmarks[i]), 5, (0,255,0))



        return img


    def find_angles_from_lines(self, lines):
        Angle = []
        Slopes = []
        for x in range(0, len(lines)):
            for x1,y1,x2,y2 in lines[x]:
                slope = (y2-y1)/(x2-x1)
                a = np.arctan(slope)# * 180.0 / np.pi;
                Angle.append(a)
                Slopes.append(slope)

        return Angle, Slopes


    def dist_lines(self,lines,slopes):
        mat = np.zeros((len(lines),len(lines),4))
        for x in range(0,len(lines)):
            for y in range(0,len(lines)):
                for x1,y1,x2,y2 in lines[x]:


                    for x3,y3,x4,y4 in lines[y]:
                        xmid1 = (x1+x2)/2
                        xmid2 = (x3+x4)/2
                        ymid1 = (y1+y2)/2
                        ymid2 = (y3+y4)/2

                        a = np.array((xmid1,ymid1))
                        b = np.array((xmid2,ymid2))
                        dist = np.linalg.norm(a-b)

                        slope1 = (y2-y1)/(x2-x1)
                        slope2 = (y4-y3)/(x4-x3)

                        slope_diff = np.absolute(slope1-slope2)

                        landmark_x = (xmid1+xmid2)/2
                        landmark_y = (ymid1+ymid2)/2


                        ang1 = np.arctan2(x2-x1,y2-y1)
                        ang2 = np.arctan2(x4-x3, y4-y3)

                        ang_diff = np.absolute(ang1 - ang2)

                        mat[x,y,0] = dist
                        mat[x,y,1] = slope_diff
                        mat[x,y,2] = np.rad2deg(ang_diff)






        return mat

    def find_pipeLineLine(self,mat, lines,display_image):

        compatible = np.logical_and(mat[:, :, 2] < 20., mat[:,:,0] < 50.)

        sumMat = np.sum(compatible, axis = 0)

        #print (-sumMat).argsort()[:2]

        bestlines_idx = (-sumMat).argsort()[0]
        #print "index best line!", bestlines_idx


        #print(lines[bestlines_idx])

        line =lines[bestlines_idx][0]
        #print "best line!", line

        cv2.line(display_image, pt1=(line[0],line[1]),pt2=(line[2],line[3]), color=(0,255,0), thickness=3)
        landmarks_x, landmarks_y, lineID = self.find_parLines(lines, mat)

        msg_frame = CvBridge().cv2_to_imgmsg(display_image)
        self.pub_pipe_img.publish(msg_frame)


        return sumMat





    def find_parLines(self,lines,mat):

        #Find all distances smaller than 80?
        #mat2 = mat[:,:,0] < 80
        xLandmarks = [0]
        yLandmarks = [0]
        lineID = []



        [a, b, c] = np.shape(mat)
        c = 0
        #print "START!!!"
        #if the slope_diff < 2 call the lines parallell ..
        for x in range(0,a):
            for y in range(0,b):
                if mat[x,y,0] < 50:
                    if mat[x,y,0] > 0:
                        if mat[x,y,1] <= 1:

                            c += 1

                            #for lines[x]:
                            for x1,y1,x2,y2 in lines[x]:
                                xmid1 = (x1+x2)/2
                                ymid1 = (y1+y2)/2

                                xLand = xmid1
                                yLand =ymid1
                            #for y in lines:
                            for x1,y1,x2,y2 in lines[y]:
                                xmid2 = (x1+x2)/2
                                ymid2 = (y1+y2)/2


                                xLand = (xmid1+ xmid2)/2 #(xmid1+xmid2)/2 #(xmid1+xmid2)/2
                                yLand = (ymid1 + ymid2)/2 #(ymid1+ymid2)/2 #(ymid1+ymid2)/2



                                #check if landmark is inside pylogon if it is add to list...

                                point = Point(xLand, yLand)
                                polygon = Polygon([(104,59),(435,41),(152,221), (410,238)])
                                #print(polygon.contains(point))
                                if polygon.contains(point):

                                    xLandmarks = np.append(xLandmarks,xLand)
                                    yLandmarks = np.append(yLandmarks, yLand)

                                    lineID.append(x)
                                    lineID.append(y)




        return xLandmarks, yLandmarks, lineID



    def cleanup(self):
        print "shutting down vision node."
        cv2.destroyAllWindows()


def main(self):


    try:
        cvBridge()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
