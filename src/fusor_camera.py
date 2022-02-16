import re
import sys
import socket
from is_msgs.common_pb2 import ConsumerList
from is_wire.core import Message, Subscription, Logger, Channel
from is_wire.rpc import ServiceProvider, LogInterceptor
from is_msgs.image_pb2 import ObjectAnnotations, ObjectAnnotation, Vertex

import numpy as np
from numpy.core.numeric import identity
from google.protobuf.json_format import Parse
from is_wire.core.utils import now
from is_msgs.robot_pb2 import RobotTaskRequest, RobotTaskReply
from is_msgs.common_pb2 import ConsumerList
import json
from math import pi, cos, sin, acos
from is_msgs.camera_pb2 import FrameTransformations, FrameTransformation

from is_msgs.common_pb2 import Pose
import math


IS_BROKER_EVENTS_TOPIC = "BrokerEvents.Consumers"
RE_CAMERA_GATEWAY_CONSUMER = re.compile("CameraGateway\\.(\\d+)\\.GetConfig")

log = Logger(name="SubscriptionManager")


def camera_parameters(file):
    camera_data = json.load(open(file))
    K = np.array(camera_data['intrinsic']['doubles']).reshape(3, 3)
    res = [camera_data['resolution']['width'],
           camera_data['resolution']['height']]
    tf = np.array(camera_data['extrinsic']['tf']['doubles']).reshape(4, 4)
    R = tf[:3, :3]
    T = tf[:3, 3].reshape(3, 1)
    dis = np.array(camera_data['distortion']['doubles'])
    return K, R, T, res, dis

def sort_corners (ids,corners):
    ind = np.argsort(ids[:,0])
    centers = np.zeros((ind.size,2))
    ids = ids[ind]
    sorted_corners = []
    
    for a in range(ind.size):
        sorted_corners.append(corners[ind[a]][:])
        centers[a,:] = np.mean(corners[ind[a]][0],axis=0) 
    return sorted_corners, centers

class SubscriptionManager:
    def __init__(self):
        self.channel = Channel("amqp://10.10.3.188:30000")
        self.subscription = Subscription(self.channel)
        self.subscription.subscribe("BrokerEvents.Consumers")
        self.cameras = []

    def run(self):
        message = self.channel.consume()
        available_cameras = []
        consumers = message.unpack(ConsumerList)
        for key, _ in consumers.info.items():
            match = RE_CAMERA_GATEWAY_CONSUMER.match(key)
            if match is None:
                continue
            available_cameras.append(int(match.group(1)))

        available_cameras.sort()

        new_cameras = list(set(available_cameras) - set(self.cameras))

        self.cameras = list(available_cameras)
        return self.cameras

class SubscriptionAruco:
    def __init__(self, camera_id):
        self.camera_id = camera_id
        self.channel = Channel("amqp://10.10.3.188:30000")
        self.subscription = Subscription(self.channel)
        self.subscription.subscribe(f"ArUco.{camera_id}.Detection")

if __name__ == "__main__":

    
    channel_publish = Channel("amqp://10.10.3.188:30000")

    #Load cameras parameters
    K1, R1, T1, res1, dis1 = camera_parameters('images/hd/camera1.json')
    K2, R2, T2, res2, dis2 = camera_parameters('images/hd/camera2.json')
    K3, R3, T3, res3, dis3 = camera_parameters('images/hd/camera3.json')
    K4, R4, T4, res4, dis4 = camera_parameters('images/hd/camera4.json')

    aurco_id = 5
    subscription_manager = SubscriptionManager()
    camera_ids = subscription_manager.run()
    arucoDetector_subscriptions = []
    for camera_id in camera_ids:
        arucoDetector_subscriptions.append(SubscriptionAruco(camera_id))

    while True:

        detected_markers = np.array([[0, 0, 0, 0]])
        centersAruco = np.array([[0, 0, 0, 0, 0, 0, 0, 0]])
        cornerAruco_1 = np.array([[0, 0, 0, 0, 0, 0, 0, 0]])
        cornerAruco_2 = np.array([[0, 0, 0, 0, 0, 0, 0, 0]])
        

        ################# consume localization (Pixel) #########################

        for arucoDetector_subscription in arucoDetector_subscriptions:
            reply = None
            try:
                reply = arucoDetector_subscription.channel.consume(timeout = 0.01)
            except socket.timeout:
                pass
            
            if reply is not None:
                objectAnnotations = reply.unpack(ObjectAnnotations)
                if objectAnnotations.objects:
                    for objectAnnotation in objectAnnotations.objects:
                        if objectAnnotation.id == aurco_id:
                            vertices = objectAnnotation.region.vertices
                            corners = np.array([[[[ vertices[0].x ,  vertices[0].y], 
                                                    [vertices[1].x,  vertices[1].y], 
                                                    [vertices[2].x,  vertices[2].y], 
                                                    [vertices[3].x,  vertices[3].y] 
                                                    ]]])
                            corners, centers = sort_corners(np.array([[aurco_id]]), corners)
                            detected_markers[0][arucoDetector_subscription.camera_id - 1] = 1  
                            if arucoDetector_subscription.camera_id == 1:
                                centersAruco[0][0] = centers[0][0]
                                centersAruco[0][1] = centers[0][1]

                                cornerAruco_1[0][0] = corners[0][0][0][0]
                                cornerAruco_1[0][1] = corners[0][0][0][1]

                                cornerAruco_2[0][0] = corners[0][0][1][0]
                                cornerAruco_2[0][1] = corners[0][0][1][1]

                            elif arucoDetector_subscription.camera_id == 2:
                                centersAruco[0][2] = centers[0][0]
                                centersAruco[0][3] = centers[0][1]

                                cornerAruco_1[0][2] = corners[0][0][0][0]
                                cornerAruco_1[0][3] = corners[0][0][0][1]

                                cornerAruco_2[0][2] = corners[0][0][1][0]
                                cornerAruco_2[0][3] = corners[0][0][1][1]

                            elif arucoDetector_subscription.camera_id == 3:
                                centersAruco[0][4] = centers[0][0]
                                centersAruco[0][5] = centers[0][1]

                                cornerAruco_1[0][4] = corners[0][0][0][0]
                                cornerAruco_1[0][5] = corners[0][0][0][1]

                                cornerAruco_2[0][4] = corners[0][0][1][0]
                                cornerAruco_2[0][5] = corners[0][0][1][1]

                            elif arucoDetector_subscription.camera_id == 4:
                                centersAruco[0][6] = centers[0][0]
                                centersAruco[0][7] = centers[0][1]   

                                cornerAruco_1[0][6] = corners[0][0][0][0]
                                cornerAruco_1[0][7] = corners[0][0][0][1]

                                cornerAruco_2[0][6] = corners[0][0][1][0]
                                cornerAruco_2[0][7] = corners[0][0][1][1]

        arucoPoints_Pixel = [centersAruco, cornerAruco_1, cornerAruco_2] 

        ############################## Reconstruction ############################# 

            # Perform reconstruction
        KRinv_1 = np.linalg.inv(np.dot(K1,R1))
        RinvT_1 = np.dot(np.linalg.inv(R1),T1)
        KRinv_2 = np.linalg.inv(np.dot(K2,R2))
        RinvT_2 = np.dot(np.linalg.inv(R2),T2)
        KRinv_3 = np.linalg.inv(np.dot(K3,R3))
        RinvT_3 = np.dot(np.linalg.inv(R3),T3)
        KRinv_4 = np.linalg.inv(np.dot(K4,R4))
        RinvT_4 = np.dot(np.linalg.inv(R4),T4)    

        arucoPoints_World = []
        i = 1 ### Gambiarra 
        detections = len(np.where(detected_markers[i-1][:] == 1)[0])   

        for centers in arucoPoints_Pixel: ## Gambiarra
            if detections > 1:
                M = np.zeros((3*detections,detections+3))
                RinvT = np.zeros((3*detections,1))
                lin = 0 
                col = 3
                # if camera 1 detected the aruco marker i
                if detected_markers[i-1][0] == 1:
                    M[lin:(lin+3),0:3] = -identity(3)
                    m = np.array([centers[i-1][0],centers[i-1][1],1])
                    M[lin:(lin+3),col] = np.dot(KRinv_1,m)
                    RinvT[lin:(lin+3),0] = RinvT_1[0:3,0]
                    lin = lin + 3
                    col = col + 1
                # if camera 2 detected the aruco marker i
                if detected_markers[i-1][1] == 1:
                    M[lin:(lin+3),0:3] = -identity(3)
                    m = np.array([centers[i-1][2],centers[i-1][3],1])
                    M[lin:(lin+3),col] = np.dot(KRinv_2,m)
                    RinvT[lin:(lin+3),0] = RinvT_2[0:3,0]
                    lin = lin + 3
                    col = col + 1
                # if camera 3 detected the aruco marker i
                if detected_markers[i-1][2] == 1:
                    M[lin:(lin+3),0:3] = -identity(3)
                    m = np.array([centers[i-1][4],centers[i-1][5],1])
                    M[lin:(lin+3),col] = np.dot(KRinv_3,m)
                    RinvT[lin:(lin+3),0] = RinvT_3[0:3,0]
                    lin = lin + 3
                    col = col + 1
                # if camera 4 detected the aruco marker i    
                if detected_markers[i-1][3] == 1:
                    M[lin:(lin+3),0:3] = -identity(3)
                    m = np.array([centers[i-1][6],centers[i-1][7],1])
                    M[lin:(lin+3),col] = np.dot(KRinv_4,m)
                    RinvT[lin:(lin+3),0] = RinvT_4[0:3,0]
                    lin = lin + 3
                    col = col + 1

                # pseudo inverse of M
                M_inv = np.linalg.pinv(M)
                # Multiplication by RinvT
                xyz = np.dot(M_inv,RinvT)
                arucoPoints_World.append(xyz)

            elif (detections == 1): 
                print('just one detection for marker ',i)
                M = np.zeros((3,3))
                RinvT = np.zeros((3,1))
                # if camera 1 detected the aruco marker i
                if detected_markers[i-1][0] == 1:
                    M[0:2,0:2] = -identity(2)
                    m = np.array([centers[i-1][0],centers[i-1][1],1])
                    M[0:3,2] = np.dot(KRinv_1,m)
                    RinvT[0:3,0] = RinvT_1[0:3,0]
                # if camera 2 detected the aruco marker i
                if detected_markers[i-1][1] == 1:
                    M[0:2,0:2] = -identity(2)
                    m = np.array([centers[i-1][2],centers[i-1][3],1])
                    M[0:3,2] = np.dot(KRinv_2,m)
                    RinvT[0:3,0] = RinvT_2[0:3,0]
                # if camera 3 detected the aruco marker i
                if detected_markers[i-1][2] == 1:
                    M[0:2,0:2] = -identity(2)
                    m = np.array([centers[i-1][4],centers[i-1][5],1])
                    M[0:3,2] = np.dot(KRinv_3,m)
                    RinvT[0:3,0] = RinvT_3[0:3,0]
                # if camera 4 detected the aruco marker i
                if detected_markers[i-1][3] == 1:
                    M[0:2,0:2] = -identity(2)
                    m = np.array([centers[i-1][6],centers[i-1][7],1])
                    M[0:3,2] = np.dot(KRinv_4,m)
                    RinvT[0:3,0] = RinvT_4[0:3,0]
                # inverse of M
                M_inv = np.linalg.inv(M)
                # Multiplication by RinvT
                xyz = np.dot(M_inv,RinvT)    
                arucoPoints_World.append(xyz) 

        ######################### pub ########################

        ######################### gambiarra #################
        if detections > 1:
            x = arucoPoints_World[0][0][0]
            y = arucoPoints_World[0][1][0]
            z = arucoPoints_World[0][2][0]
            x_p2p1 = arucoPoints_World[2][0][0] - arucoPoints_World[1][0][0]
            y_p2p1 = arucoPoints_World[2][1][0] - arucoPoints_World[1][1][0]
            roll_rad = np.arctan2(y_p2p1, x_p2p1)
            roll_deg = (180*roll_rad)/pi
            print(x, y, roll_deg)

            f = FrameTransformation()
            f.tf.doubles.append(x)
            f.tf.doubles.append(y)
            f.tf.doubles.append(z)
            f.tf.doubles.append(roll_rad)

            message = Message(content=f)
            channel_publish.publish(message, topic=f"localization.{aurco_id}.aruco")


        elif detections == 1:
            x = arucoPoints_World[0][0][0]
            y = arucoPoints_World[0][1][0]
            x_p2p1 = arucoPoints_World[2][0][0] - arucoPoints_World[1][0][0]
            y_p2p1 = arucoPoints_World[2][1][0] - arucoPoints_World[1][1][0]
            roll_rad = np.arctan2(y_p2p1, x_p2p1)
            roll_deg = (180*roll_rad)/pi
            print(x, y, roll_deg)


            f = FrameTransformation()
            f.tf.doubles.append(x)
            f.tf.doubles.append(y)
            f.tf.doubles.append(0)
            f.tf.doubles.append(roll_rad)

            message = Message(content=f)
            channel_publish.publish(message, topic=f"localization.{aurco_id}.aruco")

