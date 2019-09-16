import sys

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
    model_path = sys.argv[1]
    print('Starting Inference Worker, Specified Model Directory: ', model_path)

    rclpy.init()
    inference_worker = InferenceWorker(model_path)
    inference_worker.run()
    rclpy.spin(inference_worker)
    inference_worker.destroy_node()
    rclpy.shutdown()
