#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import time
from absl import app, flags, logging
from absl.flags import FLAGS
import numpy as np
import tensorflow as tf
from models import (
    YoloV3, YoloV3Tiny
)
from dataset import transform_images, load_tfrecord_dataset
from utils import draw_outputs

# We do this because of a bug in kinetic when importing cv2
import sys
#import subprocess
#new_proc = subprocess.Popen(["rosversion", "-d"], stdout=subprocess.PIPE)
#version_str = new_proc.communicate()[0]
#ros_version = version_str.decode('utf8').split("\n")[0]
ros_version = "kinetic"
if ros_version == "kinetic":
    try:
        sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    except Exception as ex:
        print(ex)
        print("Its already removed..../opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2
from cv_bridge import CvBridge

flags.DEFINE_string('classes', './data/new_names.names', 'path to classes file')
flags.DEFINE_string('weights', './checkpoints/yolov3_train_20.tf',
                    'path to weights file')
flags.DEFINE_boolean('tiny', False, 'yolov3 or yolov3-tiny')
flags.DEFINE_integer('size', 416, 'resize images to')
flags.DEFINE_string('output', './data/output.jpg', 'path to output image')
flags.DEFINE_integer('num_classes', 1, 'number of classes in the model')

class TensorFlowIageRecognition(object):

    def __init__(self):

        """
        python scripts/detect.py --image ./data/yolo_test_images/person_ignisbot.png
        """
        physical_devices = tf.config.experimental.list_physical_devices('GPU')
        if len(physical_devices) > 0:
            tf.config.experimental.set_memory_growth(physical_devices[0], True)


        if FLAGS.tiny:
            self.yolo = YoloV3Tiny(classes=FLAGS.num_classes)
        else:
            self.yolo = YoloV3(classes=FLAGS.num_classes)

        self.yolo.load_weights(FLAGS.weights).expect_partial()
        logging.info('weights loaded')

        self.class_names = [c.strip() for c in open(FLAGS.classes).readlines()]
        logging.info('classes loaded')

    def detect_objects_from_image(self, img_raw, save_detection=False):

        img = tf.expand_dims(img_raw, 0)
        img = transform_images(img, FLAGS.size)

        t1 = time.time()
        #boxes, scores, classes, nums = self.yolo(img)
        boxes, scores, classes, nums = self.yolo.predict(img)
        t2 = time.time()
        logging.info('time: {}'.format(t2 - t1))

        logging.info('detections:')
        objects_detected_list = []
        for i in range(nums[0]):
            logging.info('\t{}, {}, {}'.format(self.class_names[int(classes[0][i])],
                                            np.array(scores[0][i]),
                                            np.array(boxes[0][i])))
            
            objects_detected_list.append(self.class_names[int(classes[0][i])])
            

        rospy.logdebug("Result-Detection="+str(objects_detected_list))
        #img = cv2.cvtColor(img_raw.numpy(), cv2.COLOR_RGB2BGR)
        img = cv2.cvtColor(img_raw, cv2.COLOR_BGR2RGB)
        img_detection = draw_outputs(img, (boxes, scores, classes, nums), self.class_names)
        if save_detection:
            cv2.imwrite(FLAGS.output, img)
            logging.info('output saved to: {}'.format(FLAGS.output))

        return img_detection, objects_detected_list


class RosTensorFlow(object):
    def __init__(self, save_detections=False, image_rostopic=True):
        # Processing the variable to process only half of the frame's lower load
        self._save_detections = save_detections
        self._process_this_frame = True
        self._image_rostopic = image_rostopic
        self._cv_bridge = CvBridge()

        # Start Tensorflow Class
        self.tensorflow_object = TensorFlowIageRecognition()

        self._camera_image_topic = '/mira/mira/camera1/image_raw'
        self.check_image_topic_ready()
        self._sub = rospy.Subscriber(self._camera_image_topic, Image, self.callback, queue_size=1)
        self._result_pub = rospy.Publisher('result', String, queue_size=1)

        self.image_detection = rospy.Publisher('/image_detection', Image, queue_size=10)




    def check_image_topic_ready(self):
        camera_image_data = None
        while camera_image_data is None and not rospy.is_shutdown():
            try:
                camera_image_data = rospy.wait_for_message(self._camera_image_topic, Image, timeout=1.0)
                rospy.loginfo("Current "+str(self._camera_image_topic)+" READY=>")
            except Exception as ex:
                print(ex)
                rospy.logerr("Current "+str(self._camera_image_topic)+" not ready yet, retrying...")
        rospy.loginfo("Camera Sensor READY")

    def publish_results_objecets_list(self, objects_detected_list):
        result_msg = String()
        for object_name in objects_detected_list:            
            result_msg.data = object_name
            self._result_pub.publish(result_msg)

    def callback(self, image_msg):
        if (self._process_this_frame):
            rospy.logdebug("Image processing....")
            image_np = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

            img_detection, objects_detected_list = self.tensorflow_object.detect_objects_from_image(img_raw=image_np,
                                                            save_detection=self._save_detections)
            
            self.publish_results_objecets_list(objects_detected_list)

            # TODO: Publish in a ROS image topic, nt in a GUI
            if self._image_rostopic:
                #self.image_detection.publish(self._cv_bridge.cv2_to_imgmsg(img_detection, "bgr8"))
                self.image_detection.publish(self._cv_bridge.cv2_to_imgmsg(cv2.cvtColor(img_detection, cv2.COLOR_BGR2RGB), "bgr8"))            
            else:
                cv2.imshow("Image window", img_detection)
                cv2.waitKey(1)
            rospy.logdebug("Image processing....DONE")
        else:
            pass
        # We invert it
        self._process_this_frame = not self._process_this_frame

    def main(self):
        rospy.spin()

def main_action(_argv):
    rospy.init_node('search_mira_robot_node', log_level=rospy.DEBUG)
    tensor = RosTensorFlow()
    tensor.main()

if __name__ == '__main__':
    try:
        app.run(main_action)
    except SystemExit:
        pass