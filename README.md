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






