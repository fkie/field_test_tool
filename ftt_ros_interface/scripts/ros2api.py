#!/usr/bin/env python3
"""ros2api.py: ROS message interface to store relevant data in the FTT database."""

__author__ = "Johannes Pellenz, Lucas Dimartino, Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import sys
import rospy
import tf2_ros
from sensor_msgs.msg import NavSatFix, Image, CompressedImage
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from industrial_msgs.msg import RobotMode
from nav_msgs.msg import OccupancyGrid
import base64
import requests
import collections
import ruamel.yaml
import rospkg
from PIL import Image as PilImage
from cv_bridge import CvBridge, CvBridgeError
from io import BytesIO

class Ros2api:
    SEGMENT_TYPE_ITO = 1
    SEGMENT_TYPE_AUTO = 2
    SEGMENT_TYPE_TELEOP = 3
    ITO_REASON_UNASSIGNED = 11

    start_logging = False

    image_count = 0

    def __init__(self):
        print("Initiating ros2api")
        self.bridge = CvBridge()
        rospy.init_node('ros2api', anonymous=True)

        # Read absolute parameters
        self.set_logging_service = self.read_param("/set_ftt_logging_service", "/set_ftt_logging")
        self.get_logging_service = self.read_param("/get_ftt_logging_service", "/get_ftt_logging")
        self.save_params_service = self.read_param("/save_ftt_params_service", "/save_ftt_params")
        # Read relative parameters
        # (parameters read upon "start logging" service call)
        # Debug parameters
        self.save_image_dir = self.read_param("~save_image_dir", ".")

        # Create subscribers
        # (data subscribers created upon "start logging" service call)
        # Debug subscribers
        rospy.Subscriber("download_image", Int32, self.download_images)
        rospy.Subscriber("download_map", Int32, self.download_map)

        # Create service servers
        rospy.Service(self.set_logging_service, SetBool, self.set_logging_callback)
        rospy.Service(self.get_logging_service, Trigger, self.get_logging_callback)
        rospy.Service(self.save_params_service, Trigger, self.save_params_callback)
        pass

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        print("Closing ros2api.")
        pass

    def reset_variables(self):
        self.last_robot_mode = None
        self.last_ts = None
        self.last_lat = None
        self.last_lng = None
        self.last_position = None
        self.last_map = None
        self.map_sent = False

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

    def read_param(self, param_name, default_value):
        param_value = rospy.get_param(param_name, default_value)
        rospy.loginfo("Parameter: [{0}]: [{1}]".format(
            param_name, param_value))
        return param_value

    def get_params(self):
        # Read parameters
        self.use_tf = self.read_param("~params/use_tf", True)
        self.map_frame = self.read_param("~params/map_frame", "map")
        self.robot_frame = self.read_param("~params/robot_frame", "base_link")
        self.server_address = self.read_param("~params/server_address", "localhost:5000")
        self.send_pose_period = self.read_param("~params/send_pose_period", 2.0)
        self.send_map_period = self.read_param("~params/send_map_period", 2.0)
        self.image_buffer_size = self.read_param("~params/image_buffer_size", 3)
        self.image_buffer_step = self.read_param("~params/image_buffer_step", 1.0)
        
        self.robot_mode_topic = self.read_param("~topics/robot_mode", "robot_mode")
        self.gps_fix_topic = self.read_param("~topics/gps_fix", "gps_fix")
        self.local_pose_topic = self.read_param("~topics/local_pose", "local_pose")
        self.map_topic = self.read_param("~topics/map", "map")
        self.image_topic = self.read_param("~topics/image", "image_raw")
        self.image_compressed_topic = self.read_param("~topics/image_compressed", "image_compressed")
        pass

    def subscribe_robot_data(self):
        rospy.loginfo("Subscribing to robot data.")
        # Reset variables
        self.reset_variables()
        # Create image buffers
        self.raw_image_buffer = collections.deque(self.image_buffer_size*[Image()], self.image_buffer_size)
        self.compressed_image_buffer = collections.deque(self.image_buffer_size*[CompressedImage()], self.image_buffer_size)
        self.last_raw_image_time = rospy.Time.now()
        self.last_compressed_image_time = rospy.Time.now()
        # Create robot data subscribers
        self.robot_data_subscribers = []
        self.robot_data_subscribers.append(rospy.Subscriber(self.robot_mode_topic, RobotMode, self.robot_mode_callback))
        self.robot_data_subscribers.append(rospy.Subscriber(self.gps_fix_topic, NavSatFix, self.navsatfix_callback))
        self.robot_data_subscribers.append(rospy.Subscriber(self.local_pose_topic, Pose, self.pose_callback))
        self.robot_data_subscribers.append(rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback))
        self.robot_data_subscribers.append(rospy.Subscriber(self.image_topic, Image, self.image_callback))
        self.robot_data_subscribers.append(rospy.Subscriber(self.image_compressed_topic, CompressedImage, self.compressed_image_callback))
        # If using tf, create tf listener for local position
        if self.use_tf:
            self.tfBuffer = tf2_ros.Buffer(rospy.Duration(self.send_pose_period))
            self.listener = tf2_ros.TransformListener(self.tfBuffer)
        pass

    def unsubscribe_robot_data(self):
        rospy.loginfo("Unsubscribing from robot data.")
        for subscriber in self.robot_data_subscribers:
            subscriber.unregister()
        if self.use_tf:
            self.listener.unregister()
        pass

    def create_post_timers(self):
        # Create data POST timers
        self.data_post_timers = []
        self.data_post_timers.append(rospy.Timer(rospy.Duration(self.send_pose_period), self.send_pose_timer_callback))
        self.data_post_timers.append(rospy.Timer(rospy.Duration(self.send_map_period), self.send_map_timer_callback))
        pass

    def stop_post_timers(self):
        for timer in self.data_post_timers:
            timer.shutdown()
        pass

    def close_current_segment(self):
        data = {"id": "Current Segment", "action": "close"}
        try:
            resp = requests.put('http://%s/segment' % self.server_address, json=data)
            if resp.status_code == 200:
                self.last_robot_mode = None
                msg = self.print_resp_json(resp)
                return True, msg
            else:
                self.print_resp_json(resp)
                msg = "Segment already closed."
                return True, msg
        except:
            msg = "Server unavailable!"
            rospy.logerr(msg)
            return False, msg

    def set_logging_callback(self, req):
        if req.data and not self.start_logging:
            self.start_logging = True
            # Get params
            self.get_params()
            # Subscribe to robot data
            self.subscribe_robot_data()
            # Create POST timers
            self.create_post_timers()
        elif not req.data and self.start_logging:
            # Close open segment
            segment_closed, message = self.close_current_segment()
            if segment_closed:
                self.start_logging = False
                # Unsubscribe to robot data
                self.unsubscribe_robot_data()
                # Stop POST timers
                self.stop_post_timers()
            else:
                return SetBoolResponse(False, message)
        else:
            if req.data:
                rospy.logwarn("Tried to start logging, but it's already running.")
            else:
                rospy.logwarn("Tried to stop logging, but it's not running.")
        return SetBoolResponse(True, "Logging is " + ("started." if req.data else "stopped."))

    def get_logging_callback(self, req):
        # Return current logging status
        return TriggerResponse(success = self.start_logging)

    def save_params_callback(self, req):
        # Update params
        self.get_params()
        # Get the config file's path
        package_path = rospkg.RosPack().get_path('ftt_ros_interface')
        file_path = package_path + "/config/ros2api_config.yaml"
        # Load the config file
        yaml = ruamel.yaml.YAML()
        try:
            with open(file_path, "r") as stream:
                config = yaml.load(stream)
        except:
            # Error
            return TriggerResponse(False, "Error while opening config file.")
        
        # Overwrite "params" and "topics" configs with current values
        config["params"]["use_tf"] = self.use_tf
        config["params"]["map_frame"] = self.map_frame
        config["params"]["robot_frame"] = self.robot_frame
        config["params"]["server_address"] = self.server_address
        config["params"]["send_pose_period"] = self.send_pose_period
        config["params"]["send_map_period"] = self.send_map_period
        config["params"]["image_buffer_size"] = self.image_buffer_size
        config["params"]["image_buffer_step"] = self.image_buffer_step
        
        config["topics"]["robot_mode"] = self.robot_mode_topic
        config["topics"]["gps_fix"] = self.gps_fix_topic
        config["topics"]["local_pose"] = self.local_pose_topic
        config["topics"]["map"] = self.map_topic
        config["topics"]["image"] = self.image_topic
        config["topics"]["image_compressed"] = self.image_compressed_topic

        # Save the updated config to file
        try:
            with open(file_path, "w") as outfile:
                yaml.dump(config, outfile)
            # Return successfully
            return TriggerResponse(True, "FTT parameters saved to %s" % file_path)
        except:
            # Error
            return TriggerResponse(False, "Error while saving parameters to file.")

    def robot_mode_callback(self, mode):
        # Check robot mode change
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
            # Get local position data
            local_x = None
            local_y = None
            if self.use_tf:
                try:
                    trans = self.tfBuffer.lookup_transform(self.map_frame, self.robot_frame, rospy.Time())
                    local_x = trans.transform.translation.x
                    local_y = trans.transform.translation.y
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logwarn("Posting new segment without local position data.")
                except:
                    rospy.logerr("Exception: %s", sys.exc_info()[0])
            else:
                if self.last_position:
                    local_x = self.last_position.x
                    local_y = self.last_position.y
                else:
                    rospy.logwarn("Posting new segment without local position data.")
            # Check gps position data
            if not (self.last_lat and self.last_lng):
                rospy.logwarn("Posting new segment without gps position data.")
            data = {"leg_id":"Current Leg", "segment_type_id": vehicle_mode, "ito_reason_id": self.ITO_REASON_UNASSIGNED, "lng":self.last_lng, "lat": self.last_lat, "local_x": local_x, "local_y": local_y, "orig_starttime_secs": rospy.get_time()}
            try:
                resp = requests.post('http://%s/segment' % self.server_address, json=data)
                if resp.status_code == 200:
                    self.last_robot_mode = mode.val
                    self.send_image_buffer()
                self.print_resp_json(resp)
            except:
                rospy.logerr("Server unavailable!")
        pass

    def navsatfix_callback(self, data):
        self.last_ts = data.header.stamp.secs+(data.header.stamp.nsecs / 1000000000.0)
        self.last_lat = data.latitude
        self.last_lng = data.longitude
        pass

    def pose_callback(self, data):
        self.last_position = data.position
        pass

    def image_callback(self, img_msg):
        if (rospy.Time.now() - self.last_raw_image_time).to_sec() > self.image_buffer_step:
            self.raw_image_buffer.append(img_msg)
            self.last_raw_image_time = rospy.Time.now()
        pass

    def compressed_image_callback (self, compressed_img_msg):
        if (rospy.Time.now() - self.last_compressed_image_time).to_sec() > self.image_buffer_step:
            self.compressed_image_buffer.append(compressed_img_msg)
            self.last_compressed_image_time = rospy.Time.now()
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
            local_x = None
            local_y = None
            if self.use_tf:
                try:
                    trans = self.tfBuffer.lookup_transform(self.map_frame, self.robot_frame, rospy.Time())
                    local_x = trans.transform.translation.x
                    local_y = trans.transform.translation.y
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Unable to get transfrom from %s to %s within the last %s seconds", self.map_frame, self.robot_frame, self.send_pose_period)
                except:
                    print(("Exception: " + sys.exc_info()[0]))
            elif self.last_position:
                local_x = self.last_position.x
                local_y = self.last_position.y
            if local_x and local_y:
                data = {"segment_id": "Current Segment", "frame_id": self.map_frame, "x": local_x, "y": local_y, "orig_secs": rospy.get_time()}
                try:
                    resp = requests.post('http://%s/local_pose' % self.server_address, json=data)
                    if resp.status_code == 200:
                        self.last_position = None
                    self.print_resp_json(resp)
                except:
                    rospy.logerr("Server unavailable!")
        pass

    def send_image_buffer(self):
        for raw_image in self.raw_image_buffer:
            if len(raw_image.data) > 0:
                img = self.create_image_jpg(raw_image)
                time = raw_image.header.stamp.secs+(raw_image.header.stamp.nsecs / 1000000000.0)
                self.send_image(img, time)
        for compressed_image in self.compressed_image_buffer:
            if len(compressed_image.data) > 0:
                time = compressed_image.header.stamp.secs+(compressed_image.header.stamp.nsecs / 1000000000.0)
                self.send_image(compressed_image.data, time)
        pass

    def create_image_jpg(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, 'rgb8')
            pil_img = PilImage.fromarray(cv_image)
            image_string = BytesIO()
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
            pil_img = PilImage.open(BytesIO(image))
            with open(image_filename.rstrip()+"."+pil_img.format.lower(), "wb") as image_file:
                image_file.write(image)
        pass

    def create_map_jpg(self, map_msg):
        pil_img = PilImage.new("L", (map_msg.info.width, map_msg.info.height))
        pil_img.putdata([255 - point/100*255 if point >= 0 else 127 for point in map_msg.data])
        pil_img = pil_img.transpose(PilImage.Transpose.FLIP_TOP_BOTTOM)
        image_string = BytesIO()
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
                if "IntegrityError" in resp_json or "UniqueViolation" in resp_json:
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
            print(("Saving requested map image to %s" % self.save_image_dir))
            encoded_image = r_json[7]
            b64_decoded_img = encoded_image.decode("base64")
            image = base64.decodestring(b64_decoded_img)
            pil_img = PilImage.open(BytesIO(image))
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
