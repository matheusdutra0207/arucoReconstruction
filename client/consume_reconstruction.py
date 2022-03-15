from __future__ import print_function
from is_wire.core import Channel, Subscription
from is_msgs.image_pb2 import ObjectAnnotation
from is_msgs.common_pb2 import Pose
import numpy as np
import math
from math import pi, cos, sin
from is_msgs.camera_pb2 import FrameTransformation

# alteracao vinicius
import json
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from math import pi
import socket

class MapClient():
    def __init__(self,mapped_area_config):
        '''To-do, could also receive the extrinsic positions of the cameras and insert it on the map
        '''
        self.area_id = mapped_area_config['area_id']
        self.area_name = mapped_area_config['area_name']
        self.scatterplots = {}
        self.position_histories = {}
        
        self.fig,self.axis = plt.subplots(1,1)
        plt.grid(True)
        plt.show(block=False)

      
        self.plot_map(mapped_area_config['area_map'])
        self.plot_obstacles(mapped_area_config['obstacles'])
        #self.plot_debug_map(area_array,obstacles_array)
       
        
        self.scatterplots['aruco'] = None
        self.position_histories['aruco'] = np.array([])

    def plot_map(self,area_map_config):
        xmin, xmax = float('inf'),-float('inf')
        ymin, ymax = float('inf'),-float('inf')
        for key,point in area_map_config.items():
            xmin = point['x'] if xmin>point['x'] else xmin
            ymin = point['y'] if ymin>point['y'] else ymin
            xmax = point['x'] if xmax<point['x'] else xmax
            ymax = point['y'] if ymax<point['y'] else ymax
            
        self.axis.set_xlim([xmax*1.1, xmin*1.1 ])
        self.axis.set_ylim([ymax*1.1,ymin*1.1])
        room = np.array([[xmax,ymax],[xmax,ymin],[xmin,ymin],[xmin,ymax],[xmax,ymax]])
        self.axis.plot(room[:,0],room[:,1],color='y')

    def plot_obstacles(self,obstacles_config):
        for key,obstacle in obstacles_config.items():
            xmin = obstacle['x_min']
            ymin = obstacle['y_min']
            xmax = obstacle['x_max']
            ymax = obstacle['x_max']
            obs = np.array([[xmax,ymax],[xmax,ymin],[xmin,ymin],[xmin,ymax],[xmax,ymax]])        
            self.axis.plot(obs[:,0],obs[:,1],color='orange')   
        

    def start_scatter(self,position,marker = '.',color='k'):
        x,y = position
        return  self.axis.scatter(x,y,marker=marker,color=color)
    
    def start_position_history(self,position):
        x,y,z,roll = position
        return np.array([x,y,z])

    def get_position_from_trans(self,transformation):
        #transformation received is of type np, as obtained by tensor to np
        x,y,z = transformation[0,3],transformation[1,3],transformation[2,3]
        pitch = - math.asin(transformation[2,0])
        yaw = math.atan2(transformation[2,1]/math.cos(pitch),transformation[2,2]/math.cos(pitch))
        roll = math.atan2(transformation[1,0]/math.cos(pitch),transformation[0,0]/math.cos(pitch))
        return x,y,z,roll

    def update_position(self,name,position,marker = '.',color='k'):
        x,y= position
        print(f'transformed aruco: {x:.2f}, {y:.2f}')
        if self.scatterplots.get(name) is not None:
            #To-do: this has to check the size of the array, else it will grow forever, not a problem at the moment
            self.position_histories[name] = np.vstack((self.position_histories[name],np.array([x,y,0])))
            self.scatterplots[name].set_offsets(self.position_histories[name][:,:-1])
        else:
            self.position_histories[name] = self.start_position_history([x,y,None,None])
            self.scatterplots[name] = self.start_scatter([x,y],marker = marker,color=color)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()




if __name__ == '__main__':

    channel = Channel("amqp://10.10.3.188:30000")
    subscription = Subscription(channel)

    aruco_id = 5
    subscription.subscribe(topic=f"localization.{aruco_id}.aruco")

    config = json.load(open('config.json', 'r'))
    area_map = MapClient(config['mapped_area'])
    while True:
        try:
            message = channel.consume(timeout = 0.0)
            f = message.unpack(FrameTransformation)

            tf = f.tf.doubles

            x = tf[0]
            y = tf[1]
            roll_rad = tf[3]
            area_map.update_position('aruco',[x ,y],color='b')

            print(x, y, (roll_rad*180)/pi)
        except socket.timeout:
            pass
