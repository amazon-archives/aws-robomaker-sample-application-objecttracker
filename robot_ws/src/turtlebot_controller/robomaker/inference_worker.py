#!/usr/bin/env python3
"""
 Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 Permission is hereby granted, free of charge, to any person obtaining a copy of this
 software and associated documentation files (the "Software"), to deal in the Software
 without restriction, including without limitation the rights to use, copy, modify,
 merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import sys

import boto3
import yaml
from PIL import Image
import numpy as np
import tensorflow as tf

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as sensor_image

FROZEN_MODEL_LOCAL_PATH = "model.pb"
FROZEN_MODEL_S3_KEY = "model/model.pb"
AWS_REGION = "us-west-2"

TRAINING_IMAGE_SIZE = (160, 120)



def load_config(config_path):
   stream = open(config_path, 'r')
   context = yaml.safe_load(stream)

   required_items = (
       'bucket_name',
       'model_key',
       'region_name'
   )

   if not all(item in context for item in required_items):
       raise ValueError('Config file is missing required items')

   return context

def download_model_from_s3(context, dest_path):
   s3 = boto3.resource('s3', region_name=context['region_name'])
   s3.meta.client.download_file(
       Bucket=context['bucket_name'],
       Key=context['model_key'],
       Filename=dest_path)



class InferenceWorker(Node):
    def __init__(self, model_path):
        super().__init__('rl_coach')
        self.model_path = model_path

    def run(self):
        self.graph = self.load_graph()
        self.session = tf.Session(graph=self.graph, config=tf.ConfigProto(allow_soft_placement=True, log_device_placement=True))

        print('INFO: Creating publisher on /cmd_vel')
        self.ack_publisher = self.create_publisher(Twist, '/cmd_vel', 100)

        print('INFO: Creating subscriber on /camera/bgr/image_raw')
        #TODO: Need to change for raspicam
        #self.subscription = self.create_subscription(sensor_image, '/camera/bgr/image_raw', self.callback_image, 10)
        self.subscription = self.create_subscription(sensor_image, '/image', self.callback_image, 10)

        print('INFO: Finished initialization')

    def load_graph(self):
        print('Loading graph...')
        with tf.gfile.GFile(self.model_path, "rb") as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())

        with tf.Graph().as_default() as graph:
            tf.import_graph_def(graph_def, name="turtlebot")

        print('INFO: Finished loading graph')

        return graph

    def callback_image(self, raw_image):
        # Read the image
        image_data = np.array(raw_image.data)
        image = Image.frombytes('RGB', (raw_image.width, raw_image.height),
                                image_data, 'raw', 'BGR', 0, 1)
        image = image.resize(TRAINING_IMAGE_SIZE)
        image = np.array(image)

        # Get the yuma component of the image
        r, g, b = image[:, :, 0], image[:, :, 1], image[:, :, 2]
        image = 0.2989 * r + 0.5870 * g + 0.1140 * b

        image = np.expand_dims(image, 2)
        image = np.expand_dims(image, 0)

        x = self.graph.get_tensor_by_name('turtlebot/main_level/agent/main/online/network_0/observation/observation:0')
        y = self.graph.get_tensor_by_name('turtlebot/main_level/agent/main/online/network_1/ppo_head_0/policy:0')
        inferenceOutput = np.argmax(self.session.run(y, feed_dict={
            x: image
        }))

        self.takeAction(inferenceOutput)

    def takeAction(self, action):
        if action == 0:  # move left
            steering = 0.6
            throttle = 0.1
        elif action == 1:  # move right
            steering = -0.6
            throttle = 0.1
        elif action == 2:  # straight
            steering = 0.0
            throttle = 0.1
        elif action == 3:  # move left
            steering = 0.3
            throttle = 0.1
        elif action == 4:  # move right
            steering = -0.3
            throttle = 0.1
        else:  # should not be here
            raise ValueError("Invalid action")

        speed = Twist()
        speed.linear.x = throttle
        speed.angular.z = steering
        self.ack_publisher.publish(speed)

    

if __name__ == '__main__':
    source_config = sys.argv[1]
    model_path = sys.argv[2]

    context = load_config(source_config)
    download_model_from_s3(context, model_path)
    print('Successfully downloaded model to ', model_path)
    print('Starting Inference Worker, Specified Model Directory: ', model_path)

    rclpy.init()
    inference_worker = InferenceWorker(model_path)
    inference_worker.run()
    rclpy.spin(inference_worker)
    inference_worker.destroy_node()
    rclpy.shutdown()
