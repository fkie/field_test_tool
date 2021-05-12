#!/usr/bin/env python
"""ros2api.py: ROS message interface to store relevant data in the FTT database."""

__author__ = "Johannes Pellenz, Lucas Dimartino, Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import sys
import rospy
import sensor_msgs.msg
import std_msgs.msg
import industrial_msgs.msg
import base64
import requests
import json
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
from StringIO import StringIO

class Ros2api:
    SEGMENT_TYPE_ITO = 1
    SEGMENT_TYPE_AUTO = 2
    SEGMENT_TYPE_TELEOP = 3
    ITO_REASON_UNASSIGNED = 11

    last_robot_mode = None
    last_ts = None
    last_lat = 0
    last_lng = 0
    last_rgb_image = None
    last_compressed_image = None

    reported_vehicle_mode = None

    send_pose_timeout = None
    image_count = 0

    def __init__(self):
        print "Initiating ros2api"
        self.bridge = CvBridge()
        rospy.init_node('ros2db', anonymous=True)
        self.send_pose_timeout = rospy.get_param("~send_pose_timeout", 2.0)
        self.server_address = rospy.get_param("~server_address", "localhost:5000")
        rospy.Subscriber("robot_position", sensor_msgs.msg.NavSatFix, self.navsatfix_callback)
        rospy.Subscriber("robot_mode", industrial_msgs.msg.RobotMode, self.robot_mode_callback)
        rospy.Subscriber("image_raw",sensor_msgs.msg.Image, self.image_callback)
        rospy.Subscriber("image_compressed", sensor_msgs.msg.CompressedImage, self.compressed_image_callback)
        rospy.Subscriber("download_image",std_msgs.msg.Int32, self.download_images)
        rospy.Timer(rospy.Duration(self.send_pose_timeout), self.send_pose_timer_callback)
        pass

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        print "Closing ros2api."
        pass

    def robot_mode_callback(self, mode):
        if self.last_robot_mode != mode.val:
            self.last_robot_mode = mode.val
            if (mode.val == mode.MANUAL):
                self.reported_vehicle_mode = self.SEGMENT_TYPE_ITO
            else:
                if (mode.val == mode.AUTO):
                    self.reported_vehicle_mode = self.SEGMENT_TYPE_AUTO
                else:
                    rospy.logwarn("Unknown robot mode. Supported modes are MANUAL (%d) and AUTO (%d). Please check your system.", mode.MANUAL, mode.AUTO)
                    return
            rospy.loginfo("Reported vehicle mode changed to %s", "MANUAL" if self.reported_vehicle_mode == self.SEGMENT_TYPE_ITO else "AUTO")
            time_now = rospy.get_rostime()
            ts = time_now.secs + time_now.nsecs / 1000000000.0
            self.send_new_segment (ts, self.reported_vehicle_mode)
        pass

    def navsatfix_callback(self, data):
        lat = data.latitude
        lng = data.longitude
        ts = data.header.stamp.secs+(data.header.stamp.nsecs / 1000000000.0)
        self.last_ts = ts
        self.last_lat = lat
        self.last_lng = lng
        pass

    def image_callback(self, img_msg):
        self.last_rgb_image = img_msg
        pass

    def compressed_image_callback (self, compressed_img_msg):
        self.last_compressed_image = compressed_img_msg
        pass

    def send_pose_timer_callback(self, event):
        self.send_last_pose()
        pass

    def send_new_segment (self, ts, vehicle_mode, ito_reason = ITO_REASON_UNASSIGNED):
        data = {"leg_id":"Current Leg", "segment_type_id": vehicle_mode, "ito_reason_id": ito_reason, "lat": self.last_lat, "lng":self.last_lng}
        resp = requests.post('http://%s/segment' % self.server_address, json=data)
        print resp.json()
        self.send_last_image_msg()
        pass
        

    def send_last_pose (self):
        if self.last_robot_mode and self.last_ts:
            data = {"segment_id": "Current Segment", "lat": self.last_lat, "lng": self.last_lng, "pose_source_id": 1}
            resp = requests.post('http://%s/pose' % self.server_address, json=data)
            print resp.json()
            self.last_ts = None
        pass

    def send_last_image_msg(self):
        if self.last_rgb_image is not None:
            img = self.create_image_jpg(self.last_rgb_image)
            self.send_image(img)
        if self.last_compressed_image is not None:
            self.send_image(self.last_compressed_image.data)
        pass

    def create_image_jpg(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, 'rgb8')
            pil_img = Image.fromarray(cv_image)
            image_string = StringIO()
            pil_img.save(image_string, 'JPEG')
            return image_string.getvalue()
        except CvBridgeError, e:
            print e
        pass

    def send_image(self, img):
        img_encoded = base64.b64encode(img)
        data = {
            "segment_id": "Current Segment", 
            "image_filename": "rgb_img"+str(Ros2api.image_count), 
            "image_data": img_encoded,
            "description": "Automatic image capture for segment transition"
        }
        Ros2api.image_count = Ros2api.image_count + 1
        resp = requests.post('http://%s/image' % self.server_address, json=data)
        print resp.json()
        pass

    def download_images(self, int_msg):
        task = {"segment_id":int_msg.data}
        r = requests.get('http://%s/image' % self.server_address, json=task)
        image_json = r.json()
        for image_data in image_json:
            image_filename = image_data[3]
            encoded_image = image_data[4]
            b64_decoded_img = encoded_image.decode("base64")
            image = base64.decodestring(b64_decoded_img)
            pil_img = Image.open(StringIO(image))
            with open(image_filename.rstrip()+"."+pil_img.format.lower(), "wb") as image_file:
                image_file.write(image)
    
    pass

def listener():
    with Ros2api():
        rospy.spin()
    pass

if __name__ == '__main__':
    listener()
    pass
