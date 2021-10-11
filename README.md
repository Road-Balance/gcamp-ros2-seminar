## gcamp-ros2-seminar

마곡 AI 로봇 해커톤 中 `ROS 2(Robot Operating System)` 로봇 제작 기술워크숍 세미나 자료

> Seminar link : [Seoul MakerSpace G Camp](https://www.g.camp/2257)

## Running Environment

* Jetson Nano 4GB Dev Kit
* Jetpack 4.6
* ROS 2 Eloquent

# ROS 2 Basic Examples

This Repo contains mainly two parts.

* ROS 2 Basic & `rclpy`  Programming Examples => *basic_topic_pkg*
* ROS 2 autonomous Robot based on Popular RC Car => *MaRo*

### 1) ROS 2 Basic Programming0

There's 6 rclpy scripts for Node programming, And also 3 for Topic programming

1. My First ROS 2 Node.
2. Run Node periodically by `create_timer`.
3. About `rclpy.spin`.
4. ROS 2 Node Composition.
5. Node Composition with `create_timer`.
6. ROS 2 Node logger level.

**How to Run**
```
ros2 run basic_topic_pkg example_node_<number>
```

Moreover, There's 3 rclpy scripts for ROS 2 Topic programming.

1. Publish `cmd_vel` message to `turtlesim` through topic publisher
2. Subscribe turtlesim's `pose` message and analize about message type.
3. Small Project, Make `turtle2` mimic `turtle1` using publisher & subscriber both.

**How to Run**
```
ros2 run basic_topic_pkg topic_pub_node
ros2 run basic_topic_pkg topic_sub_node
ros2 run basic_topic_pkg topic_pub_sub_node
```
## MaRo

* picture

## Setup - Hardware

* ongoing

## Setup - Software

* ongoing

## Run Examples

* ongoing

### TODO

- [ ] Fully support for Nav 2