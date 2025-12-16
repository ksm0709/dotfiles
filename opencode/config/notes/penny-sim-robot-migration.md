# Penny Sim Robot Migration

## How to Migrate New Robot Type to Penny-Sim

Based on the changes for `amr_rect_300L_0.1.0` ("Add cart carrier to penny-sim"):

1.  **Create Configuration Directory**

    Create a new directory for the robot type at `ROS/simulation/<robot_type>_configs` and add a `config.yaml`.

    **Example: `ROS/simulation/amr_rect_300L_0.1.0_configs/config.yaml`**

    ```yaml
    mode: test # [ prod | sbr_prod | test | dev ]

    robot:
      type: amr_rect_300L_0.1.0

    controller:
      name: test #host-name
    organizer:
      name: test #host-name

    universe:
      name: test #host-name
      preset: test # [ prod | rc | test | dev | local ]

    systemer:
      name: test #host-name

    processer:
      name: test #host-name
    ```

2.  **Add Overrides**

    Create an `override/` subdirectory and add necessary override YAML files (hardware, software, controller, etc.).

    **Example: `ROS/simulation/amr_rect_300L_0.1.0_configs/override/hardware.yaml`**

    ```yaml
    hardware:
      topic_checker:
        sensor:
          astra:
            enable: false
            frequency_check_enable: false
          # ... disable other unused sensors ...
      astra:
        front:
          serial: "1"
        down:
          serial: "2"
        up:
          serial: "3"
      urdf: ${include astra_calibration.yaml}
      odom: ${include wheel_calibration.yaml}
    ```

3.  **Update URDF**

    Ensure the robot's URDF/Xacro file supports the Gazebo namespace argument (`gz_ns`) for multi-robot simulation.

    **Example: `ROS/pennybot_description/urdf/amr_rect_300L_0.1.0.urdf.xacro`**

    ```xml
    <xacro:arg name="gz_ns" default=""/>

    <!-- ... -->

    <xacro:sick_scan_generic_calibration
      name="${lidar_front_name}"
      parent="${lidar_front_name}_fixed_link"
      lidar_topic="$(arg gz_ns)/$(bearconfig hardware.lidar.front.topic)"
      robot_namespace="$(arg gz_ns)">
      <!-- ... -->
    </xacro:sick_scan_generic_calibration>
    ```

4.  **Update Launch Files**

    Modify `ROS/pennybot/launch/robot_multimaster.launch` to include new topic patterns if the robot has new sensors.

    **Example: `ROS/pennybot/launch/robot_multimaster.launch`**

    ```xml
    <arg name="topic_list" default="[
        '/R$(arg robot_num)/lidar/scan',
        '/R$(arg robot_num)/lidar_f/scan',
        '/R$(arg robot_num)/*/scan',  <!-- Added wildcard for generic scan topics -->
        '/R$(arg robot_num)/*/depth/points', <!-- Added wildcard for generic depth points -->
        '/R$(arg robot_num)/astra/depth/points',
        ...
    ```

5.  **Code Adjustments**

    Update C++ components like `multimaster_topic_synchronizer` to handle specific sensors for the new robot type.

    **Example: `ROS/simulation/multimaster_topic_synchronizer/src/multimaster_topic_synchronizer.cpp`**

    ```cpp
    // Adding new LiDAR publishers
    lidar_top_ros_pub_ =
        nh->advertise<sensor_msgs::LaserScan>("/lidar_top/scan", 1);
    lidar_top_ros_sub_ = nh->subscribe<sensor_msgs::LaserScan>(
        "/" + gazebo_ns_ + "/lidar_top/scan", 1,
        std::bind(&MultimasterTopicSynchronizer::PublishLidarScan, this,
                  std::placeholders::_1, "lidar_t_link", lidar_top_ros_pub_));

    // Adding new Gemini camera publishers
    gemini_up_ros_pub_ =
        nh->advertise<sensor_msgs::PointCloud2>("/gemini_up/depth/points", 1);
    gemini_up_ros_sub_ = nh->subscribe<sensor_msgs::PointCloud2>(
        "/" + gazebo_ns_ + "/gemini_up/depth/points", 1,
        std::bind(&MultimasterTopicSynchronizer::PublishAstraPointCloud, this,
                  std::placeholders::_1, "gemini_up_depth_optical_frame",
                  gemini_up_ros_pub_));
    ```

Tags: #penny-sim #migration #robot-type #changelog
