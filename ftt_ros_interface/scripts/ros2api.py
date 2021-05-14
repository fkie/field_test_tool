#!/usr/bin/env python
"""ros2api.py: ROS message interface to store relevant data in the FTT database."""

__author__ = "Johannes Pellenz, Lucas Dimartino, Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import sys
import rospy
import tf2_ros
import sensor_msgs.msg
import std_msgs.msg
import industrial_msgs.msg
import nav_msgs.msg
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
    last_lat = None
    last_lng = None
    last_rgb_image = None
    last_compressed_image = None
    last_map = None
    map_sent = False

    image_count = 0

    def __init__(self):
        print("Initiating ros2api")
        self.bridge = CvBridge()
        rospy.init_node('ros2api', anonymous=True)

        self.send_pose_timeout = rospy.get_param("~send_pose_timeout", 2.0)
        self.send_map_timeout = rospy.get_param("~send_map_timeout", 2.0)
        self.server_address = rospy.get_param("~server_address", "localhost:5000")
        self.save_image_dir = rospy.get_param("~save_image_dir", ".")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")

        rospy.Subscriber("robot_position", sensor_msgs.msg.NavSatFix, self.navsatfix_callback)
        rospy.Subscriber("robot_mode", industrial_msgs.msg.RobotMode, self.robot_mode_callback)
        rospy.Subscriber("image_raw",sensor_msgs.msg.Image, self.image_callback)
        rospy.Subscriber("image_compressed", sensor_msgs.msg.CompressedImage, self.compressed_image_callback)
        rospy.Subscriber("map", nav_msgs.msg.OccupancyGrid, self.map_callback)
        rospy.Subscriber("download_image",std_msgs.msg.Int32, self.download_images)
        rospy.Subscriber("download_map",std_msgs.msg.Int32, self.download_map)
        rospy.Timer(rospy.Duration(self.send_pose_timeout), self.send_pose_timer_callback)
        rospy.Timer(rospy.Duration(self.send_map_timeout), self.send_map_timer_callback)
        
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(self.send_pose_timeout))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        pass

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        print("Closing ros2api.")
        pass

    def print_resp_json(self, resp):
        print_msg = ""
        try:
            resp_json = resp.json()
            if "message" in resp_json:
                resp_json = resp_json["message"]
            if "[ERROR]" in resp_json:
                print_msg = resp_json.split("[ERROR]",1)[1][1:]
                rospy.logerr(print_msg)
            elif "[WARN]" in resp_json:
                print_msg = resp_json.split("[WARN]",1)[1][1:]
                rospy.logwarn(print_msg)
            elif "[INFO]" in resp_json:
                print_msg = resp_json.split("[INFO]",1)[1][1:]
                rospy.loginfo(print_msg)
            else:
                print_msg = resp_json
                rospy.loginfo(print_msg)
        except:
            print_msg = "Server internal error."
            rospy.logerr(print_msg)
        return print_msg

    def robot_mode_callback(self, mode):
        if self.last_robot_mode != mode.val:
            if (mode.val == mode.MANUAL):
                vehicle_mode = self.SEGMENT_TYPE_ITO
            else:
                if (mode.val == mode.AUTO):
                    vehicle_mode = self.SEGMENT_TYPE_AUTO
                else:
                    rospy.logwarn("Unknown robot mode. Supported modes are MANUAL (%d) and AUTO (%d). Please check your system.", mode.MANUAL, mode.AUTO)
                    return
            rospy.loginfo("Reported vehicle mode changed to %s", "MANUAL" if vehicle_mode == self.SEGMENT_TYPE_ITO else "AUTO")
            local_x = None
            local_y = None
            try:
                trans = self.tfBuffer.lookup_transform(self.map_frame, self.robot_frame, rospy.Time())
                local_x = trans.transform.translation.x
                local_y = trans.transform.translation.y
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Posting new segment without local position data.")
            except:
                rospy.logerr("Exception: %s", sys.exc_info()[0])
            data = {"leg_id":"Current Leg", "segment_type_id": vehicle_mode, "ito_reason_id": self.ITO_REASON_UNASSIGNED, "lng":self.last_lng, "lat": self.last_lat, "local_x": local_x, "local_y": local_y, "orig_starttime_secs": rospy.get_time()}
            try:
                resp = requests.post('http://%s/segment' % self.server_address, json=data)
                if resp.status_code == 200:
                    self.last_robot_mode = mode.val
                    self.send_last_image_msg()
                self.print_resp_json(resp)
            except:
                rospy.logerr("Server unavailable!")
        pass

    def navsatfix_callback(self, data):
        self.last_ts = data.header.stamp.secs+(data.header.stamp.nsecs / 1000000000.0)
        self.last_lat = data.latitude
        self.last_lng = data.longitude
        pass

    def image_callback(self, img_msg):
        self.last_rgb_image = img_msg
        pass

    def compressed_image_callback (self, compressed_img_msg):
        self.last_compressed_image = compressed_img_msg
        pass

    def map_callback (self, map_msg):
        self.last_map = map_msg
        pass

    def send_pose_timer_callback(self, event):
        if self.last_robot_mode:
            self.send_last_gps_pose()
            self.send_last_local_pose()
        pass

    def send_map_timer_callback(self, event):
        if self.last_map:
            if self.map_sent:
                self.update_map(self.last_map)
            else:
                self.send_new_map(self.last_map)
        pass

    def send_last_gps_pose(self):
        if self.last_lng and self.last_lat:
            data = {"segment_id": "Current Segment", "lat": self.last_lat, "lng": self.last_lng, "pose_source_id": 1, "orig_secs": self.last_ts}
            try:
                resp = requests.post('http://%s/pose' % self.server_address, json=data)
                if resp.status_code == 200:
                    self.last_lng = None
                    self.last_lat = None
                self.print_resp_json(resp)
            except:
                rospy.logerr("Server unavailable!")
        pass

    def send_last_local_pose(self):
        if self.map_sent:
            try:
                trans = self.tfBuffer.lookup_transform(self.map_frame, self.robot_frame, rospy.Time())
                data = {"segment_id": "Current Segment", "frame_id": self.map_frame, "x": trans.transform.translation.x, "y": trans.transform.translation.y, "orig_secs": rospy.get_time()}
                try:
                    resp = requests.post('http://%s/local_pose' % self.server_address, json=data)
                    self.print_resp_json(resp)
                except:
                    rospy.logerr("Server unavailable!")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Unable to get transfrom from %s to %s within the last %s seconds", self.map_frame, self.robot_frame, self.send_pose_timeout)
            except:
                print("Exception:", sys.exc_info()[0])
        pass

    def send_last_image_msg(self):
        if self.last_rgb_image is not None:
            img = self.create_image_jpg(self.last_rgb_image)
            time = self.last_rgb_image.header.stamp.secs+(self.last_rgb_image.header.stamp.nsecs / 1000000000.0)
            self.send_image(img, time)
        if self.last_compressed_image is not None:
            time = self.last_compressed_image.header.stamp.secs+(self.last_compressed_image.header.stamp.nsecs / 1000000000.0)
            self.send_image(self.last_compressed_image.data, time)
        pass

    def create_image_jpg(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, 'rgb8')
            pil_img = Image.fromarray(cv_image)
            image_string = StringIO()
            pil_img.save(image_string, 'JPEG')
            return image_string.getvalue()
        except CvBridgeError as e:
            print(e)
        pass

    def send_image(self, img, time):
        img_encoded = base64.b64encode(img)
        data = {
            "segment_id": "Current Segment", 
            "image_filename": "rgb_img"+str(Ros2api.image_count), 
            "image_data": img_encoded,
            "description": "Automatic image capture for segment transition",
            "orig_secs": time
        }
        Ros2api.image_count = Ros2api.image_count + 1
        try:
            resp = requests.post('http://%s/image' % self.server_address, json=data)
            self.print_resp_json(resp)
        except:
            rospy.logerr("Server unavailable!")
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

    def create_map_jpg(self, map_msg):
        pil_img = Image.new("L", (map_msg.info.width, map_msg.info.height))
        pil_img.putdata([255 - point/100*255 if point >= 0 else 127 for point in map_msg.data])
        pil_img = pil_img.transpose(Image.FLIP_TOP_BOTTOM)
        image_string = StringIO()
        pil_img.save(image_string, 'JPEG')
        return image_string.getvalue()

    def send_new_map(self, map_msg):
        map_img_encoded = base64.b64encode(self.create_map_jpg(map_msg))
        data = {
            "shift_id": "Current Shift",
            "frame_id": map_msg.header.frame_id,
            "width": map_msg.info.width,
            "height": map_msg.info.height,
            "resolution": map_msg.info.resolution,
            "origin_x": map_msg.info.origin.position.x,
            "origin_y": map_msg.info.origin.position.y,
            "image_data": map_img_encoded,
            "orig_secs": map_msg.header.stamp.secs+(map_msg.header.stamp.nsecs / 1000000000.0)
        }
        try:
            resp = requests.post('http://%s/map_image' % self.server_address, json=data)
            resp_json = self.print_resp_json(resp)
            if resp.status_code == 200:
                self.map_sent = True
                self.last_map = None
            else:
                if "IntegrityError" in resp_json:
                    self.update_map(map_msg)
        except:
            rospy.logerr("Server unavailable!")
        pass

    def update_map(self, map_msg):
        map_img_encoded = base64.b64encode(self.create_map_jpg(map_msg))
        data = {
            "shift_id": "Current Shift",
            "image_data": map_img_encoded,
            "frame_id" : map_msg.header.frame_id,
            "width" : map_msg.info.width,
            "height" : map_msg.info.height,
            "resolution" : map_msg.info.resolution,
            "origin_x" : map_msg.info.origin.position.x,
            "origin_y" : map_msg.info.origin.position.y,
            "orig_secs": map_msg.header.stamp.secs+(map_msg.header.stamp.nsecs / 1000000000.0)
        }
        resp = requests.put('http://%s/map_image' % self.server_address, json=data)
        if resp.status_code == 200:
            self.map_sent = True
            self.last_map = None
        self.print_resp_json(resp)
        pass

    def download_map(self, int_msg):
        data = {"shift_id": int_msg.data}
        r = requests.get('http://%s/map_image' % self.server_address, json=data)
        r_json = r.json()
        if r.status_code == 200:
            print("Saving requested map image to %s" % self.save_image_dir)
            encoded_image = r_json[7]
            b64_decoded_img = encoded_image.decode("base64")
            image = base64.decodestring(b64_decoded_img)
            pil_img = Image.open(StringIO(image))
            with open(self.save_image_dir + "/map_" + str(int_msg.data) + "."+pil_img.format.lower(), "wb") as image_file:
                image_file.write(image)
        else:
            print(r_json)
        pass
    
    pass

def listener():
    with Ros2api():
        rospy.spin()
    pass

if __name__ == '__main__':
    listener()
    pass
