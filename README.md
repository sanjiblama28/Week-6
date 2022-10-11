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






