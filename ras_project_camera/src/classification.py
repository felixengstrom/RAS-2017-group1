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
import operator


class classification(object):
    def __init__(self):
               
        self.sub_object_detected = rospy.Subscriber('/camera/image/object_detected', Bool, self.object_detected_cb)
        self.sub_image = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_cb, queue_size = 1)
        self.bridge = CvBridge()
        rospy.loginfo("subscribed to /camera/image/image_raw")
        #print "subscribed to /camera/image/image_raw"
                
    def object_detected_cb(self, msg):
        self.flag = msg.data
        rospy.loginfo("object flag %i", flag)

    def image_cb (self, img):

        rospy.loginfo("in image callback")        
        # Input 
        # convert from msg to cv::Mat
        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        # convert form cv:Mat to PIL
        pil_im = fromarray(cv_image)
        size = (200, 200)
        pil_im = pil_im.resize(size, resample=0)
        x = image.img_to_array(pil_im)
        print x.shape
        x = np.expand_dims(x, axis=0)
        x = preprocess_input(x)
        
        # Prediction 
        model = load_model('VGG16_trained_model.h5')
        rospy.loginfo('load the model')
        #model.summary()
        preds = model.predict(x)
        rospy.loginfo('Predicted object is %d', preds[0][0])
        classes = ['green_cude', 'green_hollow_cube']
        pred = preds[0]
        index, value = max(enumerate(pred), key=operator.itemgetter(1))
        clas = classes[index]
        result = (clas, value)
        rospy.loginfo('Predicted object is %s', result[0])
        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)

    def main (self):
        
        rospy.loginfo("in main")
        

    
if __name__ == '__main__':
    rospy.init_node('classification')
    object_classification = classification()
    object_classification.main
    rospy.spin()