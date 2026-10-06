"""One fixed RGB-D camera, exact pairing and explicit observation freshness."""
import json
import time
from pathlib import Path
import cv2
cv2.setNumThreads(1)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from .geometry import detect


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def pair(rgb, depth, info, last_pair):
    common=rgb.keys() & depth.keys()
    if not common or info is None:return None
    key=max(common)
    if key==last_pair:return None
    r,rw=rgb[key];d,dw=depth[key]
    if time.monotonic()-min(rw,dw)>1:raise ValueError('Camera data stale')
    return key,r,d,min(rw,dw)


class Perception(Node):
    def __init__(self):
        super().__init__('bai3_perception')
        self.config=json.loads((Path(get_package_share_directory('hri_bai3_environment'))/'config/scene.json').read_text())
        self.rgb={};self.depth={};self.info=None;self.last_pair=None;self.last_received=0.
        self.bridge=CvBridge()
        self.publisher=self.create_publisher(String,'/bai3/table_state',1)
        self.create_subscription(Image,'/bai3/camera/image',lambda m:self.cache(self.rgb,m),qos_profile_sensor_data)
        self.create_subscription(Image,'/bai3/camera/depth_image',lambda m:self.cache(self.depth,m),qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/bai3/camera/camera_info',self.on_info,qos_profile_sensor_data)
        self.create_timer(.05,self.process)

    def on_info(self,msg):self.info=msg

    def cache(self,store,msg):
        store[stamp(msg)]=(msg,time.monotonic())
        while len(store)>12:del store[min(store)]

    def process(self):
        state={'source':'rgbd','frame_id':self.config['frame'],'objects':{},'errors':{}}
        try:
            result=pair(self.rgb,self.depth,self.info,self.last_pair)
            if not result:
                if time.monotonic()-self.last_received<1:return
                raise ValueError('Camera synchronized RGB-D stale/unavailable; no fallback')
            key,rgb,depth,received=result
            if not (rgb.header.frame_id==depth.header.frame_id==self.info.header.frame_id):
                raise ValueError('Camera RGB/depth/camera_info frame mismatch')
            self.last_pair=key;self.last_received=received
            state['objects'],state['errors']=detect(self.bridge.imgmsg_to_cv2(rgb,'rgb8'),
                self.bridge.imgmsg_to_cv2(depth,'32FC1'),list(self.info.k),self.config['camera'])
            state['stamp']=key;state['sensor_frame']=rgb.header.frame_id
            for item in state['objects'].values():
                item.update(stamp=key,age_s=max(0.,time.monotonic()-received),view='fixed_camera')
        except Exception as error:state['error']=str(error)
        self.publisher.publish(String(data=json.dumps(state)))


def main():
    rclpy.init();node=Perception()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
