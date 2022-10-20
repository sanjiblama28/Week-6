# Managing Dependencies with rosdep

## How do I use the rosdep tool?

We are prepared to use the utility now that we have a basic understanding of rosdep, package.xml, and rosdistro. First, if using rosdep for the first time, it must be initialized using:

```
sudo rosdep init
rosdep update
```

![image](https://user-images.githubusercontent.com/92040822/194984951-6c15ca19-ef70-459a-bc1d-c3dcbed8acb9.png)


We can finally use rosdep install to install dependencies. This is often called once across a workspace that contains numerous packages in order to install all dependencies. If the workspace's root contained the source-code-containing directory src, a call for that might look like the following.

```
rosdep install --from-paths src -y --ignore-src
```
![image](https://user-images.githubusercontent.com/92040822/194985049-f97687a2-a070-432b-8d50-15b0e4f318d9.png)

# Creating an action

## Prerequisites

Colcon and ROS 2 must be installed.

Establish a workspace and a package called "action tutorials interfaces":

(Remember to source first from your ROS 2 installation.)

```
mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces
```

![image](https://user-images.githubusercontent.com/92040822/194985781-d285bf6d-dc69-4898-b207-e77806cc20ab.png)

## Tasks
## 1. Defining an action

Actions are described in .action files with the following format:

```
# Request
---
# Result
---
# Feedback
```
Create a directory called "action" in our ROS 2 package called "action tutorials interfaces":

```
cd action_tutorials_interfaces
mkdir action
```

![image](https://user-images.githubusercontent.com/92040822/194997859-cf7b8875-7ee6-4fb1-ad5d-61ea9c9b3ec8.png)


Make a file called Fibonacci in the action directory. action that includes the following information:

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

## 2. Building an action

The definition of the new Fibonacci action type must be sent to the pipeline that generates Rosidl code before we can use it in our code.

The following lines need be added to our CMakeLists.txt before the ament package() line in the action tutorials interfaces to achieve this:

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

We must also include the necessary dependencies in our package.xml file:

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

The fact that action definitions contain additional metadata means that we must rely on action msgs (e.g. goal IDs).

The package containing the definition of the Fibonacci action should now be able to be built:

```
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build
```

![image](https://user-images.githubusercontent.com/92040822/194997429-6b430f6e-8bbd-4bb3-9f57-33c69ba07dec.png)

We are done!

Action types will often start with the term "action" and the package name. As a result, our new action will be referred to by its complete name, action tutorials interfaces/action/Fibonacci.

Using the command line tool, we can verify that our action was built successfully:

```
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```

![image](https://user-images.githubusercontent.com/92040822/194998140-8b4ab337-de02-4b82-8eed-09df6c031cbf.png)


The definition of the Fibonacci action should appear on the screen.

# Writing an action server and client

## Prerequisites

You will require both the Fibonacci.action interface from the previous tutorial, "Creating an action," and the action tutorials interfaces package.

## Tasks

## 1. Writing an action server

You should create a new file in your home directory called fibonacci action server.py and add the following code to it:

```
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```

Let's attempt to run our action server:

```
python3 fibonacci_action_server.py
```

![image](https://user-images.githubusercontent.com/92040822/195005805-079536ff-680e-41f3-8b1f-52c91067fd85.png)

We can communicate a goal via the command line interface to another terminal:

```
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

![image](https://user-images.githubusercontent.com/92040822/195005945-5bed8f77-710a-4cc2-b45e-e13fcd590e2c.png)

You should see the logged message "Executing goal..." followed by a notice that the goal state was not established in the terminal that is running the action server. The aborted state is assumed by default if the goal handle state is not set in the execute callback.

The succeed() method on the goal handle can be used to show that the goal was successful:

```
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        goal_handle.succeed()

        result = Fibonacci.Result()
        return result
```

You should see the goal completed with the status SUCCEED if you restart the action server and send another goal at this point. 

![image](https://user-images.githubusercontent.com/92040822/195006346-83e3bae1-0ce7-4693-97e1-ea8ce72be268.png)

![image](https://user-images.githubusercontent.com/92040822/195006367-2b341169-9964-4b03-b992-d3ee7165785a.png)

Let's now make sure that our target execution computes and returns the specified Fibonacci sequence:

```
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        sequence = [0, 1]



        for i in range(1, goal_handle.request.order):

            sequence.append(sequence[i] + sequence[i-1])


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = sequence

        return result
```

We compute the sequence, assign it to the result message field, and then proceed to the return.

Send another goal and restart the action server. The aim should be completed with the expected results in order.

![image](https://user-images.githubusercontent.com/92040822/195006747-a1500020-d012-434b-9ad3-87ba9987323e.png)

![image](https://user-images.githubusercontent.com/92040822/195006803-99357e2b-2932-4885-8205-e26e372d6f1a.png)

## 1.2 Publishing feedback

The sequence variable will be swapped out, and the sequence will now be stored in a feedback message. We publish the feedback message and then fall asleep after each update of the feedback message in the for-loop for impact:

```
import time


import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        feedback_msg = Fibonacci.Feedback()

        feedback_msg.partial_sequence = [0, 1]


        for i in range(1, goal_handle.request.order):

            feedback_msg.partial_sequence.append(

                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = feedback_msg.partial_sequence

        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```

Utilizing the command line tool with the --feedback option after restarting the action server, we can verify that feedback has now been published:

```
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

![image](https://user-images.githubusercontent.com/92040822/195008024-7aed6b72-3e4a-4baa-ae36-de3bee6fdd54.png)


![image](https://user-images.githubusercontent.com/92040822/195007345-05b776e6-bec4-47ab-bd78-f714953600e0.png)

## 2. Writing an action client

We'll limit the action client to just one file as well. Then, open a new file and name it fibonacci action client.py. Add the following boilerplate code to the new file:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
```

Let's test our action client by first launching the earlier-built action server:

```
python3 fibonacci_action_server.py
```

Run the action client in an other terminal.

```
python3 fibonacci_action_client.py
```

As the action server completes the goal, the following messages should be printed:

```
[INFO] [fibonacci_action_server]: Executing goal...
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
# etc.
```

![image](https://user-images.githubusercontent.com/92040822/195008493-54dd71ba-08eb-4e15-a8fd-607641d65326.png)

The action client should begin and complete as soon as possible. We currently have a working action client, but we receive no feedback or results.

## 2.1 Getting a result

We must first obtain a goal handle for the goal that we sent. The result can then be requested using the goal handle.

The full code for this example is provided here:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

Go ahead and attempt to run our Fibonacci action client while an action server is running in a separate terminal!

```
python3 fibonacci_action_client.py
```

![image](https://user-images.githubusercontent.com/92040822/195009217-a65fdda3-72f8-46c5-b863-a95166ea9dce.png)

You should be able to see the goal being accepted and the outcome in the logs.

## 2.2 Getting feedback

We can send goals to our action client. Nice! However, it would be wonderful to hear some input regarding the goals we transmit from the action server.

Here is the whole code for this illustration:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

Everything is ready for us. Your screen should display feedback if we run our action client.

```
python3 fibonacci_action_client.py
```

![image](https://user-images.githubusercontent.com/92040822/195010081-8356976d-ace7-4f85-acc7-2d20318cd105.png)

# Composing multiple nodes in a single process


## Discover available components

Run the following commands in a shell to see what components are registered and accessible in the workspace:

```
ros2 component types
```

The terminal will provide a list of every component that is available:

```
(... components of other packages here)
composition
  composition::Talker
  composition::Listener
  composition::Server
  composition::Client
```

![image](https://user-images.githubusercontent.com/92040822/196695754-20bd1682-0e54-408c-8a42-db48a92739d6.png)
  
## Run-time composition using ROS services with a publisher and subscriber

Start the component container in the first shell.

```
ros2 run rclcpp_components component_container
```

Open the second shell, then use the ros2 command line tools to confirm if the container is active:

```
ros2 component list
```

You ought to see the component's name:

```
/ComponentManager
```

![image](https://user-images.githubusercontent.com/92040822/196696246-e7e9202a-1a14-4ab3-8ecb-a5b4a5f0bb64.png)

Load the talker component into the second shell (see the talker source code):

```
ros2 component load /ComponentManager composition composition::Talker
```

The command will return the unique ID of the loaded component as well as the node name:

```
Loaded component 1 into '/ComponentManager' container node as '/talker'
```
![image](https://user-images.githubusercontent.com/92040822/196696591-c9d24ce0-ca5a-464a-9468-ad576f604616.png)

Messages indicating that the component was loaded and that a message was published should now appear in the first shell.

![image](https://user-images.githubusercontent.com/92040822/196696795-ce644651-75ba-4b4d-a068-0a8528473160.png)

Run a different command in the second shell to load the listener component (see the listener source code):

```
ros2 component load /ComponentManager composition composition::Listener
```

Terminal will return:

```
Loaded component 2 into '/ComponentManager' container node as '/listener'
```

![image](https://user-images.githubusercontent.com/92040822/196697102-034a3256-6639-4c72-93bb-c3228e761c14.png)

Now, the container's state may be examined using the ros2 command-line tool:

```
ros2 component list
```

The outcome will be as follows:

```
/ComponentManager
   1  /talker
   2  /listener
```

![image](https://user-images.githubusercontent.com/92040822/196697455-c2d18f17-164f-4540-a740-c94286c1bbc7.png)

The first shell should now provide repeated output for each message that was received.

## Run-time composition using ROS services with a server and client

The first shell contains:

```
ros2 run rclcpp_components component_container
```

In the second shell (see the source code for the server and client):

```
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
```
![image](https://user-images.githubusercontent.com/92040822/196700307-a1e303ca-a221-4ff3-b434-395f8b3355c9.png)

In this scenario, the client requests something from the server, which then processes the request and sends back an answer, which the client then outputs.

## Compile-time composition using ROS services

This demonstration demonstrates how the same shared libraries can be used to create an executable that runs numerous components from a single file. The talker and listener, as well as the server and client, are all included in the executable.

In the shell call (see source code):

```
ros2 run composition manual_composition
```

![image](https://user-images.githubusercontent.com/92040822/196701189-3235f758-0cf1-4056-ba68-b336e2bc5987.png)

This should demonstrate repeated messages from both pairs, including the talker and listener, server, and client.

## Run-time composition using dlopen

This demonstration offers a different approach to run-time composition by building a generic container process and explicitly passing the libraries to load without utilizing ROS APIs. One instance of each "rclcpp::Node" class will be created in each open library's source code by the procedure.

```
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
```

![image](https://user-images.githubusercontent.com/92040822/196701548-94c73072-f111-49ef-b73c-9d042a0d1f9f.png)

## Composition using launch actions

While the command line tools can be helpful for troubleshooting and debugging component setups, it is frequently more practical to launch a group of components at once. We can make use of the ros2 launch feature to automate this process.

```
ros2 launch composition composition_demo.launch.py
```

![image](https://user-images.githubusercontent.com/92040822/196701949-072ec24a-d9f9-41a1-bac9-0006ecac1081.png)

# Advanced Topics

We may look at more sophisticated issues now that the fundamental operation of the components has been completed.

## Unloading components

Start the component container in the first shell.

```
ros2 run rclcpp_components component_container
```

Using the ros2 command line tools, confirm the container is active:

```
ros2 component list
```

You ought to see the component's name:

```
/ComponentManager
```

![image](https://user-images.githubusercontent.com/92040822/196702620-268dfad0-cc21-4731-acfb-6039b53bfec0.png)

Within the second shell (see talker source code). Both the node name and the component's distinctive ID will be returned by the command.

```
ros2 component load /ComponentManager composition composition::Talker
ros2 component load /ComponentManager composition composition::Listener
```

![image](https://user-images.githubusercontent.com/92040822/196702977-1352f25b-9f67-402b-bf8d-235e6c2b06e4.png)

The node can be removed from the component container by using its special ID.

```
ros2 component unload /ComponentManager 1 2
```

The terminal should return:

```
Unloaded component 1 from '/ComponentManager' container
Unloaded component 2 from '/ComponentManager' container
```

![image](https://user-images.githubusercontent.com/92040822/196703265-ea574fef-5475-4d20-95cd-04c278b5c5b2.png)

Verify that the talker and listener's repeated messages have stopped in the first shell.

## Remapping container name and namespace

Standard command line parameters can be used to remap the name and namespace of the component manager:

```
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns
```

Components can be loaded in a second shell by using the modified container name:

```
ros2 component load /ns/MyContainer composition composition::Listener
```

![image](https://user-images.githubusercontent.com/92040822/196704015-2ee12101-fcbb-4afb-babd-0bde5dbbe0b5.png)

![image](https://user-images.githubusercontent.com/92040822/196704105-431a27db-dd25-431f-9213-4de11f920802.png)

## Remap component names and namespaces

Arguments to the load command can be used to change component names and namespaces.

Start the component container in the first shell.

```
ros2 run rclcpp_components component_container
```

Several illustrations of name and namespace remapping.

Node name to remap:

```
ros2 component load /ComponentManager composition composition::Talker --node-name talker2
```

Remap namespace:

```
ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns
```

Remap both:

```
ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2
```

Now use ros2 command line utility:

```
ros2 component list
```

In the console you should see corresponding entries:

```
/ComponentManager
   1  /talker2
   2  /ns/talker
   3  /ns2/talker
```

![image](https://user-images.githubusercontent.com/92040822/196704739-27d7c9b9-861c-4ff6-b9fc-757c95829fb6.png)

![image](https://user-images.githubusercontent.com/92040822/196704864-f163488d-c1b7-4f38-9f01-44b24568552a.png)

## Passing parameter values into components

Passing arbitrary parameters to the node as it is being built is supported by the ros2 component load command-line. Here are some examples of how to use this functionality:

```
ros2 component load /ComponentManager image_tools image_tools::Cam2Image -p burger_mode:=true
```

![image](https://user-images.githubusercontent.com/92040822/196705455-d317ca6e-5118-4d81-9a22-f47106ee2e27.png)

## Passing additional arguments into components

Specific settings can be passed to the component manager using the ros2 component load command-line to be used while building the node. Using intra-process communication to instantiate a node is currently the sole supported command-line option. The following are some uses for this functionality:

```
ros2 component load /ComponentManager composition composition::Talker -e use_intra_process_comms:=true
```

![image](https://user-images.githubusercontent.com/92040822/196705580-627ccdb4-0a6d-4c6e-87b8-0889d515724b.png)

# Creating launch files

## 1. Setup

Organize your launch files in a new directory by creating it:

```
mkdir launch
```

## 2. Write the launch file

Utilize the turtlesim package and associated executables to create a ROS 2 launch file.
The following code should be copied and pasted into the launch/turtlesim mimic launch.py file:

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

![image](https://user-images.githubusercontent.com/92040822/196708973-1cb57155-fc2b-4deb-ab23-16c7a2fe6a0d.png)

## 3. ROS2 launch

Enter the directory you previously created, then issue the following command to start the launch file you produced above:

```
cd launch
ros2 launch turtlesim_mimic_launch.py
```
The following [INFO] messages letting you know which nodes your launch file has begun appear in two turtlesim windows:

```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [11714]
[INFO] [turtlesim_node-2]: process started with pid [11715]
[INFO] [mimic-3]: process started with pid [11716]
```

![image](https://user-images.githubusercontent.com/92040822/196709379-78ff3b4e-049d-492d-950f-84a8965785a8.png)

Open a fresh terminal and issue the ros2 topic pub command on the /turtlesim1/turtle1/cmd_vel topic to move the first turtle in order to witness the system in action:

```
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

The two turtles will appear to be traveling in unison.

![image](https://user-images.githubusercontent.com/92040822/196709891-62c85dd4-199e-4880-936f-3c83e7c86203.png)


## 4. Introspect the system with rqt_graph

Open a new terminal and run rqt graph while the system is still operating to better understand the connections between the nodes in your launch file.

Execute this command:

```
rqt_graph
```

![image](https://user-images.githubusercontent.com/92040822/196710417-a83aaf8a-9823-485f-82c3-bdf321c85916.png)

![image](https://user-images.githubusercontent.com/92040822/196711277-7ab17e0d-f549-4fb2-965d-54b5eac66c1d.png)

![image](https://user-images.githubusercontent.com/92040822/196711387-a46765f7-20f0-4e45-86a0-d87982a470df.png)

The /turtlesim1/sim node is subscribed to the /turtlesim1/turtle1/cmd_vel topic on the left, to which a hidden node (the ros2 topic pub command you executed) is publishing data. The remainder of the graph demonstrates what was previously stated: mimic publishes to the velocity command topic of /turtlesim2/sim and subscribes to the posture topic of /turtlesim1/sim.

## 5. Summary

Launch files make it easier to execute complicated systems with numerous nodes and intricate configurations. Using the ros2 launch command, launch files created in Python, XML, or YAML can be executed.

# Integrating launch files into ROS2 packages

## 1. Create a package
Make a living environment for the package at your workspace:

```
mkdir -p launch_ws/src
cd launch_ws/src
```

```
ros2 pkg create py_launch_example --build-type ament_python
```

![image](https://user-images.githubusercontent.com/92040822/196713918-e6bfbe1a-e7d3-4ece-b569-776a643519c9.png)

## 2. Creating the structure to hold launch files

The directory that houses your package should have the following structure for Python packages:

```
src/
  py_launch_example/
    package.xml
    py_launch_example/
    resource/
    setup.py
    setup.cfg
    test/
```

We must tell Python's setup tools about our launch files using the data files setup option in order for colcon to discover the launch files.

Inside our setup.py file:

```
import os
from glob import glob
from setuptools import setup

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```

## 3. Writing the Launch 

Create a new launch file with the name my_script_launch.py inside of your launch directory.

The generate launch description() function, which produces a launch.LaunchDescription() that the ros2 launch verb can utilize, should be defined in your launch file.

```
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
  ])
```

## 4. Building and running the launch file

Go to the workspace's top level and construct it:

```
colcon build
```

![image](https://user-images.githubusercontent.com/92040822/196971132-d9b4a896-8ee7-4522-b433-f1833038fcde.png)

You should be able to run the launch file as follows once the colcon build has been successful and you've sourced the workspace:

```
ros2 launch py_launch_example my_script_launch.py
```

# Using substitutions

## 1. Create and setup the package

Make a launch tutorial package of build_type ament_python.

```
ros2 pkg create launch_tutorial --build-type ament_python
```

Make a directory called launch inside of the package.

```
mkdir launch_tutorial/launch
```

![image](https://user-images.githubusercontent.com/92040822/196972159-af6efe98-1868-4f99-985b-7a9686baea84.png)

Changes should be made to the package's setup.py file to ensure the installation of the launch files:

```
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```

## 2. Parent launch file

Make a file called example_main.launch.py and place it in the launch folder of the launch tutorial package.

```
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```

## 3. Substitutions example launch file

Create a file called example substitutions.launch.py in the same folder now.

```
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```

## 4. Build the package

Go to the workspace's root and create the package:

```
colcon build
```

![image](https://user-images.githubusercontent.com/92040822/196974298-1b77d237-310d-464a-998f-ac0f9d7e805d.png)


## Launching example

Now you can use the ros2 launch command to launch the example_main.launch.py file.

```
ros2 launch launch_tutorial example_main.launch.py
```

It will accomplish the following:

- Start a blue-background turtlesim node.

- Produce a second turtle

- Purple should be used instead.

- If the provided background r parameter is 200 and the use provided red argument is True, change the color to pink after two seconds.

![image](https://user-images.githubusercontent.com/92040822/196980509-753a8f41-000d-492e-9e31-9b226fb52590.png)

![image](https://user-images.githubusercontent.com/92040822/196980609-bde01bbb-8997-4fd2-8924-f2ed8a62cb84.png)

## Modifying launch arguments

If you want to modify the default launch arguments, you may either do so in the example main.launch.py launch arguments dictionary.

```
ros2 launch launch_tutorial example_substitutions.launch.py --show-args
```

This displays the launch file's possible arguments along with their default values.

```
Arguments (pass arguments as '<name>:=<value>'):

    'turtlesim_ns':
        no description given
        (default: 'turtlesim1')

    'use_provided_red':
        no description given
        (default: 'False')

    'new_background_r':
        no description given
        (default: '200')
```

![image](https://user-images.githubusercontent.com/92040822/196980903-c9221afe-6523-4cdf-8d50-65369b05b175.png)

The launch file can now receive the following arguments:

```
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

![image](https://user-images.githubusercontent.com/92040822/196981113-5ec82b0b-87b4-4b0f-bb39-7ff82615a7c2.png)

![image](https://user-images.githubusercontent.com/92040822/196981266-ad252736-7c62-414d-8742-63596f8cfea6.png)

## Summary

This tutorial taught me how to use replacements in launch files and how to make reusable launch files using them.

# Using Event Handlers

## Event handler example launch file

We added a new file called "example_event_handlers.launch.py" to the same directory, i.e. inside the "launch" folder of the "launch tutorial" package.

```
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawn_turtle,
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    TimerAction(
                        period=2.0,
                        actions=[change_background_r_conditioned],
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=turtlesim_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])
```

## Build the package

Go to the root of the workspace, and build the package:

```
colcon build
```

## Launching example

The example_event_handlers.launch.py file can now be launched using the ros2 launch command after you have finished creating and sourcing the workspace.

```
ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```
The results of this will be:

Start a turtlesim node with a blue background while spawning a second turtle and changing the color to purple. If the provided background_r argument is 200 and the use_provided_red argument is True, change the color to pink after two seconds. Shut down the launch file when the turtlesim window is closed.

Additionally, when the turtlesim node starts, the spawn action, the change_background_r action and the change_background_r_conditioned action is executed respectively and the turtlesim node leaves and the launch process is prompted to shut down  which results in notifications to belogged to the console.

![image](https://user-images.githubusercontent.com/92040822/196988780-b6446fd4-3df1-4dfa-b24d-eefb3d0be555.png)

![image](https://user-images.githubusercontent.com/92040822/196988908-43053242-9f9e-4b1f-b020-e22b769220e1.png)













s









