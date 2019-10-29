# Object Tracker

This Sample Application can train a reinforcement learning model to make a TurtleBot WafflePi to follow a TurtleBot burger. It can then deploy and run the learned model to a real-life TurtleBot WafflePi via AWS RoboMaker.

_RoboMaker sample applications include third-party software licensed under open-source licenses and is provided for demonstration purposes only. Incorporation or use of RoboMaker sample applications in connection with your production workloads or a commercial products or devices may affect your legal rights or obligations under the applicable open-source licenses. Source code information can be found [here](https://s3.console.aws.amazon.com/s3/buckets/robomaker-applications-us-east-1-72fc243f9355/object-tracker/?region=us-east-1)._

Keywords: Reinforcement learning, AWS, RoboMaker

![object-tracker-world.jpg](docs/images/object-tracker-world.jpg)


## Requirements

- ROS Dashing - To run the simulation locally (other distributions of ROS may work, but they have not been tested)
- Gazebo 9 (optional) - To run the simulation locally
- TurtleBot WafflePi (optional) - To run the trained reinforcement learning model in the real world
- An AWS S3 bucket - To store the trained reinforcement learning model
- AWS RoboMaker - To run the simulation and to deploy the trained model to the robot


## AWS Account Setup
  
### AWS Credentials
You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services. You may find [AWS Configuration and Credential Files](https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html) helpful.

### AWS Permissions
To train the reinforcement learning model in simulation, you need an IAM role with the following policy. You can find instructions for creating a new IAM Policy
[here](https://docs.aws.amazon.com/IAM/latest/UserGuide/access_policies_create.html#access_policies_create-start). In the JSON tab paste the following policy document:

```
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Action": [
                "cloudwatch:PutMetricData",
                "logs:CreateLogGroup",
                "logs:CreateLogStream",
                "logs:PutLogEvents",
                "logs:DescribeLogStreams",
                "s3:Get*",
                "s3:List*",
                "s3:Put*",
                "s3:DeleteObject"
            ],
            "Effect": "Allow",
            "Resource": "*"
        }
    ]
}
```


## Usage (without RoboMaker)

If you plan on using this application with AWS RoboMaker, you can find more detailed instructions in the "Usage with RoboMaker" section.

### Training the model

#### Building the simulation bundle

```bash
cd simulation_ws
rosws update
rosdep install --from-paths src --ignore-src -r -y
colcon build
colcon bundle
```

#### Running the simulation

The following environment variables must be set when you run your simulation:

- `MARKOV_PRESET_FILE` - Defines the hyperparameters of the reinforcement learning algorithm. This should be set to `object_tracker.py`.
- `MODEL_S3_BUCKET` - The name of the S3 bucket in which you want to store the trained model.
- `MODEL_S3_PREFIX` - The path where you want to store the model.
- `ROS_AWS_REGION` - The region of the S3 bucket in which you want to store the model.
- `AWS_ACCESS_KEY_ID` - The access key for the role you created in the "AWS Permissions" section
- `AWS_SECRET_ACCESS_KEY` - The secret access key for the role you created in the "AWS Permissions" section
- `AWS_SESSION_TOKEN` - The session token for the role you created in the "AWS Permissions" section

Once the environment variables are set, you can run local training using the roslaunch command

```bash
source /usr/share/gazebo/setup.sh
source simulation_ws/install/setup.sh
ros2 launch object_tracker_simulation local_training.launch.py
```

### Evaluating the model

#### Building the simulation bundle

You can reuse the bundle from the training phase again in the simulation phase.

#### Running the simulation

The evaluation phase requires that the same environment variables be set as in the training phase. Once the environment variables are set, you can run
evaluation using the roslaunch command

```bash
source /usr/share/gazebo/setup.sh
source simulation_ws/install/setup.sh
ros2 launch object_tracker_simulation evaluation.launch.py
```

### Deploying the model

#### Building the robot bundle

*IMPORTANT*
You must build the bundle for the same architecture that you plan on running the application on. The easiest way to do this is either by building on
the TurtleBot itself or using a cross-build solution, like the one provided with AWS RoboMaker.

Before you build the robot workspace, you must edit the file `robot_ws/src/object_tracker_robot/config/model_config.yaml` to include the location
of your trained model.

Build your application. If your model is not in a private bucket, the build step requires AWS credentials for the bucket you wish to use.
One way to do this is by setting the AWS_ACCESS_KEY_ID and AWS_SECRET_ACCESS_KEY environment variables. For more information on how to get
your AWS credentials, see [this page](https://docs.aws.amazon.com/general/latest/gr/aws-sec-cred-types.html).
```bash
cd robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

If you are not running on the same computer you are using to build, you must also bundle your application using `colcon bundle --bundle-version 1`.

#### Running the model on the TurtleBot

If you did not build on the robot, you must now copy the bundle to the robot you wish to run on, either by using RoboMaker's deployment features or by
copying using `scp` or `rsync`.

Support for the Raspberry Pi camera in ROS Dashing on Ubuntu is currently limited. We recommend using a USB webcam instead.

Give the ROS nodes permission to the use the camera by running the following command before attempting to start the robot application on the robot:
```bash
sudo chmod 666 /dev/video0
```

Once the bundle has been uploaded to the target TurtleBot WafflePi, ssh into the TurtleBot and run

```bash
export BUNDLE_CURRENT_PREFIX=<bundle location>
source $BUNDLE_CURRENT_PREFIX/setup.sh
ros2 launch object_tracker_robot main.launch.py run_usb_cam:=True
```

Your TurtleBot WafflePi should now be track and move towards any other TurtleBot you put in front of it! For the best results, your real-life environment should match
your simulated environment as closely as possible. You can try tweaking your simulated environment by adding randomization, lighting, textures, or even to
have the TurtleBot try to track a different object.


## Usage with RoboMaker

### Adding a trust policy to your role

If you are using RoboMaker to train the model, you need to add the following trust policy to the role you created in the "AWS Permissions" section. Instructions on how to
modify a role can be found [here](https://docs.aws.amazon.com/IAM/latest/UserGuide/id_roles_manage_modify.html#roles-managingrole-editing-console).

```
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Effect": "Allow",
            "Principal": {
                "Service": "robomaker.amazonaws.com"
            },
            "Action": "sts:AssumeRole"
        }
    ]
}
```

### Building and running the simulation application

You can build and bundle the simulation application the same way you would locally. This produces the artifact `simulation_ws/bundle/output.tar`. 
You'll need to upload these to an s3 bucket, then you can use these files to
[create a robot application](https://docs.aws.amazon.com/robomaker/latest/dg/create-robot-application.html),
[create a simulation application](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-application.html),
and [create a simulation job](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-job.html) in RoboMaker.

When you create the simulation job in RoboMaker, you must assign a public IP to the simulation, give the simulation subnets and a
security group, and set the following environment variables in your simulation application:

- `MARKOV_PRESET_FILE` - Defines the hyperparameters of the reinforcement learning algorithm. This should be set to `object_tracker.py`.
- `MODEL_S3_BUCKET` - The name of the S3 bucket in which you want to store the trained model.
- `MODEL_S3_PREFIX` - The path where you want to store the model.
- `ROS_AWS_REGION` - The region of the S3 bucket in which you want to store the model.

Finally, the launch command for the simulation application is as follows:

```bash
source simulation_ws/install/setup.sh
ros2 launch object_tracker_simulation local_training.launch.py
```

### Evaluating the model

To evaluate your model in RoboMaker, clone the job you used to train and change the launch command of the simulation application to the following:

```bash
ros2 launch object_tracker_simulation evaluation.launch.py
```

### Building the robot bundle

Once the model is trained, you can build a bundle to deploy to the robot. If you are using the RoboMaker Development Environment, you can use the
following commands to create a Docker container that performs cross-builds:
```bash
cd /opt/robomaker/cross-compilation-dockerfile
sudo bin/build_image.bash
cd ~/environment/ObjectTracker/robot_ws
docker run -v $(pwd):/robot_ws -v ~/.aws:/root/.aws -it ros-cross-compile:armhf
cd robot_ws
```

Before you build the robot workspace, you must edit the file `robot_ws/src/object_tracker_robot/config/model_config.yaml` to include the location
of your trained model.

Build and bundle your application.
```bash
cd /robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
colcon bundle
```

### Deploying the bundle to a robot

Once the application is bundled, you can create a robot application and deploy the bundle to your robot. For more information,
see [this page](https://docs.aws.amazon.com/robomaker/latest/dg/gs-deploy.html).

You must run the following command before deploying the Robot Application to the robot.
```bash
sudo chmod 777 /dev/video0
```

## Seeing your robot learn

As the reinforcement learning model improves, the reward function will increase. You can see the graph of this reward function at

All -> AWSRoboMakerSimulation -> Metrics with no dimensions -> Metric Name -> ObjectTrackerRewardPerEpisode

You can think of this metric as an indicator into how well your model has been trained. If the graph has plateaus, then your robot has finished learning.

![object-tracker-metrics.png](docs/images/object-tracker-metrics.png)


## Troubleshooting

###### The robot does not look like it is training

The training algorithm has two phases. The first is when the reinforcement learning model attempts to navigate the robot towards
its target, while the second is when the algorithm uses the information gained in the first phase to update the model. In the second
phase, no new commands are sent to the TurtleBot, meaning it will appear as if it is stopped, spinning in circles, or drifting off
aimlessly.


## License

Most of this code is licensed under the MIT-0 no-attribution license. However, the sagemaker_rl_agent package is
licensed under Apache 2. See LICENSE.txt for further information.


## How to Contribute

Create issues and pull requests against this Repository on Github
