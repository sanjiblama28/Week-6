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

##Compile-time composition using ROS services

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

















