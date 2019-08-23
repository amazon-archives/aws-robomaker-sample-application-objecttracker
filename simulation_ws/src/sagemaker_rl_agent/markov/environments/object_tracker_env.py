from __future__ import print_function

import time

# only needed for fake driver setup
import boto3
# gym
import gym
import numpy as np
from gym import spaces
from PIL import Image
import os
import random
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as sensor_image
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

TRAINING_IMAGE_SIZE = (160, 120)

# REWARD ENUM
CRASHED = 0
NO_PROGRESS = -1
REWARD_CONSTANT = 10.0
MAX_STEPS = 1000000

# SLEEP INTERVALS
SLEEP_AFTER_RESET_TIME_IN_SECOND = 0.5
SLEEP_BETWEEN_ACTION_AND_REWARD_CALCULATION_TIME_IN_SECOND = 0.1
SLEEP_WAITING_FOR_IMAGE_TIME_IN_SECOND = 0.01

### Gym Env ###
class TurtleBot3ObjectTrackerAndFollowerEnv(gym.Env):
    def __init__(self):

        screen_height = TRAINING_IMAGE_SIZE[1]
        screen_width = TRAINING_IMAGE_SIZE[0]

        self.on_track = 0
        self.progress = 0
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.distance_from_center = 0
        self.distance_from_border_1 = 0
        self.distance_from_border_2 = 0
        self.steps = 0
        self.progress_at_beginning_of_race = 0
        self.burger_x = 0
        self.burger_y = 0

        # actions -> steering angle, throttle
        self.action_space = spaces.Box(low=np.array([-1, 0]), high=np.array([+1, +1]), dtype=np.float32)

        # given image from simulator
        self.observation_space = spaces.Box(low=0, high=255,
                                            shape=(screen_height, screen_width, 3), dtype=np.uint8)

        #ROS initialization
        rclpy.init()
        self.node = rclpy.create_node('rl_coach')

        self.ack_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.gazebo_model_state_service = self.node.create_client(SetEntityState, '/set_entity_state')

        #Subscribe to ROS topics and register callbacks
        self.node.create_subscription(Odometry, '/odom', self.callback_position, 10)
        self.node.create_subscription(sensor_image, '/camera/image_raw', self.callback_image, 10)
        self.aws_region = os.environ.get("ROS_AWS_REGION", "us-east-1")

        self.reward_in_episode = 0
        self.steps = 0
        self.last_distance_of_turtlebot = sys.maxsize

    def reset(self):
        print('Total Reward Reward=%.2f' % self.reward_in_episode,
              'Total Steps=%.2f' % self.steps)
        self.send_reward_to_cloudwatch(self.reward_in_episode)

        self.reward = None
        self.done = False
        self.next_state = None
        self.image = None
        self.steps = 0
        self.prev_progress = 0
        self.reward_in_episode = 0

        self.send_action(0.0, 0.0) # set the throttle to 0
        self.turtlebot3_reset()

        self.infer_reward_state()
        return self.next_state

    def turtlebot3_reset(self):
        while not self.gazebo_model_state_service.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')

        self.x = 0.0
        self.y = 0.0

        # Put the turtlebot waffle at (0, 0)
        entityState = SetEntityState.Request()
        entityState.state.pose.position.z = 0.0
        entityState.state.pose.orientation.x = 0.0
        entityState.state.pose.orientation.y = 0.0
        entityState.state.pose.orientation.z = 0.0
        entityState.state.pose.orientation.w = 0.0
        entityState.state.twist.linear.x = 0.0
        entityState.state.twist.linear.y = 0.0
        entityState.state.twist.linear.z = 0.0
        entityState.state.twist.angular.x = 0.0
        entityState.state.twist.angular.y = 0.0
        entityState.state.twist.angular.z = 0.0
        entityState.state.name = 'turtlebot3_waffle_pi'
        entityState.state.pose.position.x = self.x
        entityState.state.pose.position.y = self.y
        self.gazebo_model_state_service.call_async(entityState)

        self.burger_x = 3.5
        self.burger_y = random.uniform(-1, 1)

        # Put the turtlebot burger at (2, 0)
        entityState = SetEntityState.Request()
        entityState.state.pose.position.z = 0.0
        entityState.state.pose.orientation.x = 0.0
        entityState.state.pose.orientation.y = 0.0
        entityState.state.pose.orientation.z = 0.0
        entityState.state.pose.orientation.w = random.uniform(0, 3)
        entityState.state.twist.linear.x = 0.0
        entityState.state.twist.linear.y = 0.0
        entityState.state.twist.linear.z = 0.0
        entityState.state.twist.angular.x = 0.0
        entityState.state.twist.angular.y = 0.0
        entityState.state.twist.angular.z = 0.0
        entityState.state.name = 'turtlebot3_burger'
        entityState.state.pose.position.x = self.burger_x
        entityState.state.pose.position.y = self.burger_y
        self.gazebo_model_state_service.call_async(entityState)

        self.last_distance_of_turtlebot = sys.maxsize

        time.sleep(SLEEP_AFTER_RESET_TIME_IN_SECOND)

    def step(self, action):
        #initialize rewards, next_state, done
        self.reward = None
        self.done = False
        self.next_state = None

        steering = float(action[0])
        throttle = float(action[1])
        self.steps += 1
        self.send_action(steering, throttle)
        time.sleep(SLEEP_BETWEEN_ACTION_AND_REWARD_CALCULATION_TIME_IN_SECOND)
        self.infer_reward_state()

        info = {} #additional data, not to be used for training
        return self.next_state, self.reward, self.done, info

    def callback_image(self, data):
        self.image = data

    def callback_position(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

    def send_action(self, steering, throttle):
        speed = Twist()
        speed.linear.x = throttle
        speed.angular.z = steering
        self.ack_publisher.publish(speed)

    def infer_reward_state(self):
        #Wait till we have a image from the camera
        while not self.image:
            rclpy.spin_once(self.node)
        rclpy.spin_once(self.node)
        image_data = np.array(self.image.data)
        image = Image.frombuffer('RGB', (self.image.width, self.image.height),
                                image_data,'raw', 'BGR', 0, 1)
        image = image.resize(TRAINING_IMAGE_SIZE)
        state = np.array(image)
        rclpy.spin_once(self.node)
        x = self.x
        y = self.y

        distance_of_turtlebot = math.sqrt((x - self.burger_x) * (x - self.burger_x)
                                          + (y - self.burger_y) * (y - self.burger_y))
        done = False

        reward = 0

        if distance_of_turtlebot < self.last_distance_of_turtlebot:
            self.last_distance_of_turtlebot = distance_of_turtlebot
            reward = REWARD_CONSTANT / (distance_of_turtlebot * distance_of_turtlebot)
            if distance_of_turtlebot < 0.2:
                done = True

        if distance_of_turtlebot > 5:
            done = True

        self.reward_in_episode += reward
        print('Step No=%.2f' % self.steps,
              'Reward=%.2f' % reward,
              'Distance of bot=%f' % distance_of_turtlebot)

        self.reward = reward
        self.done = done
        self.next_state = state

    def send_reward_to_cloudwatch(self, reward):
        session = boto3.session.Session()
        cloudwatch_client = session.client('cloudwatch', region_name=self.aws_region)
        cloudwatch_client.put_metric_data(
            MetricData=[
                {
                    'MetricName': 'ObjectTrackerRewardPerEpisode',
                    'Unit': 'None',
                    'Value': reward
                },
            ],
            Namespace='AWSRoboMakerSimulation'
        )

class TurtleBot3ObjectTrackerAndFollowerDiscreteEnv(TurtleBot3ObjectTrackerAndFollowerEnv):
    def __init__(self):
        TurtleBot3ObjectTrackerAndFollowerEnv.__init__(self)

        # actions -> straight, left, right
        self.action_space = spaces.Discrete(5)

    def step(self, action):

        # Convert discrete to continuous
        if action == 0:  # move left
            steering = 0.6
            throttle = 0.1
        elif action == 1:  # move right
            steering = -0.6
            throttle = 0.1
        elif action == 2:  # straight
            steering = 0
            throttle = 0.1
        elif action == 3:  # move left
            steering = 0.3
            throttle = 0.1
        elif action == 4:  # move right
            steering = -0.3
            throttle = 0.1
        else:  # should not be here
            raise ValueError("Invalid action")

        continous_action = [steering, throttle]

        return super().step(continous_action)


class TurtleBot3ObjectTrackerAndFollowerMultiDiscreteEnv(TurtleBot3ObjectTrackerAndFollowerEnv):
    def __init__(self):
        TurtleBot3ObjectTrackerAndFollowerEnv.__init__(self)

        # actions -> straight, left, right
        self.action_space = spaces.Discrete(10)

    def step(self, action):

        # Convert discrete to continuous
        if action == 0:  # straight
            throttle = 0.3  # 0.5
            steering = 0
        elif action == 1:
            throttle = 0.7
            steering = 0
        elif action == 2:
            throttle = 1.0
            steering = 0
        elif action == 3:
            throttle = 0.1
            steering = 1
        elif action == 4:  # move left
            throttle = 0.1
            steering = -1
        elif action == 5:  # move left
            throttle = 0.3
            steering = 0.75
        elif action == 6:  # move left
            throttle = 0.3
            steering = -0.75
        elif action == 7:  # move right
            throttle = 0.5
            steering = 0.5
        else:  # action == 4
            throttle = 0.5
            steering = -0.5

        continous_action = [steering, throttle]

        return super().step(continous_action)
