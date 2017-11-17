#!/usr/bin/env python
import rospy
import roslib
import sys, time
import cv2
from cv_bridge import CvBridge
from PIL.Image import fromarray
from PIL import Image
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from keras.applications.vgg16 import preprocess_input
from keras.preprocessing import image
from keras.models import load_model
#from keras.applications.vgg16 import decode_predictions
import numpy as np
import random
import operator


class classification(object):
    def __init__(self):

        self.flag = 0
        self.buffer_size = 52428800
        self.sub_object_detected = rospy.Subscriber('/camera/image/object_detected', Bool, self.object_detected_cb)
        self.sub_image = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_cb, queue_size = 1, buff_size=self.buffer_size)
        self.object_frame = rospy.Publisher('/camera/image/object_frame', Image, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("subscribed to /camera/image/image_raw")

    def object_detected_cb(self, msg):
        self.flag = msg.data
        rospy.loginfo("object flag %i", self.flag)
    
    def image_cb (self, img):
        rospy.loginfo("object flaggg %i", self.flag)
        WINDOW_SIZES = [i for i in range(20, 160, 20)]
        best_box = None
        best_box_prob = -np.inf        
        #r = rospy.Rate(100)
        #while not rospy.is_shutdown():
        if (self.flag ==1):
            self.model = load_model('VGG16_trained_model.h5')  
            rospy.loginfo('load the model')
            # Input 
            # convert from msg to cv::Mat
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
            for win_size in window_sizes:
                for top in range(0, cv_image.shape[0] - win_size + 1, step):
                    for left in range(0, cv_image.shape[1] - win_size + 1, step):
                    # compute the (top, left, bottom, right) of the bounding box
                	box = (top, left, top + win_size, left + win_size)
                    # crop the original image
                	cropped_cv_image = cv_image[box[0]:box[2], box[1]:box[3]]
                    print('predicting for box %r' % (box, ))
            		# convert form cv:Mat to PIL
                    pil_im = fromarray(cropped_cv_image)
                    size = (200, 200)
                    pil_im = pil_im.resize(size, resample=0)
                    x = image.img_to_array(pil_im)
                    print x.shape
                    x = np.expand_dims(x, axis=0)
                    x = preprocess_input(x)
                    preds = self.model.predict(x)
                    pred = preds[0]
                    index, value = max(enumerate(pred), key=operator.itemgetter(1))
                    if pred[index] > best_box_prob:
                        best_box = box
                        best_box_prob = pred[index]
                    print('best bounding box %r' % (best_box, ))
                    classes = ['green_cube', 'green_hollow_cylinder']
                    clas = classes[index]
                    result = (clas, value)
                    rospy.loginfo('Predicted object is %s', result[0])
            		#cv2.imshow("Image window", cv_image)
            		#cv2.waitKey(3)
            		#r.sleep()
            img_frame = cv2.rectangle(cv_image,(best_box[0],best_box[1]),(best_box[2],best_box[3]),(0,255,0),5)
            msg_frame = CvBridge().cv2_to_imgmsg(img_frame)
            object_frame.publish(msg_frame, "RGB8")
    
if __name__ == '__main__':
    rospy.init_node('classification')
    object_classification = classification()
    rospy.spin()
