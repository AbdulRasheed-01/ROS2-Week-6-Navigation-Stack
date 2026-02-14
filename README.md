# ROS2-Week-6-Navigation-Stack
🎯 Learning Objectives
By the end of this week, you will be able to:

✅ Understand Nav2 architecture and components

✅ Set up navigation for differential drive robots

✅ Configure global and local planners

✅ Implement AMCL for robot localization

✅ Create and save maps using SLAM

✅ Handle dynamic obstacles and costmaps

✅ Implement custom behaviors and recovery

📚 Theory Content

Core Components:

Component	    |      Package	               |           Purpose

Map Server	  |      nav2_map_server	       |           Provides static map data

AMCL	        |      nav2_amcl	             |           Adaptive Monte Carlo Localization

Planner Server	|    nav2_planner	           |           Global path planning

Controller Server |	  nav2_controller	       |           Local trajectory planning

Behavior Server	  |  nav2_behaviors	         |           Recovery behaviors

Costmap 2D	      |  nav2_costmap_2d	       |           Obstacle representation

Lifecycle         |  Manager	nav2_lifecycle_manager	|  Node lifecycle management

6.1 Costmaps Explained

Costmap Layers:

    Static Layer    ──►  Pre-known obstacles from map
    Obstacle Layer  ──►  Sensor data (LiDAR, depth camera)
    Inflation Layer ──►  Expands obstacles with cost decay

Cost Values:

    0      ← Free space (No obstacle)
    1-127  ← Inflated area (Cost decreases with distance)
    128    ← Unknown space
    129-252 ← Lethal obstacle (Planner avoids)
    253    ← Inscribed (Robot footprint collides)
    254    ← Lethal (Direct collision)
    255    ← No information


6.2 Planners Comparison


Planner	    |               Type	         |           Best For	          |      Pros	        |    Cons

Navfn	    |               Global	         |           Simple environments  | 	 Fast, stable	|    No dynamic reconfig

Smac Planner |	            Global	         |           Complex environments |	     Optimal paths, |    2D/SE2	Slower

Theta*	     |              Global	         |           Any-angle paths	  |      Shorter paths	|    More computation

DWB	Local	 |              Differential drive	|        Configurable,        |      smooth	        |    Many parameters

TEB	Local	 |              Car-like robots	    |        Time-optimal,        |      kinematics	    |    Oscillations

Regulated Pure Pursuit |    Local	            |        General purpose	  |      Stable, simple	|    Less optimal


6.3 Localization Methods

AMCL (Adaptive Monte Carlo Localization):

Particle filter-based localization

Adapts number of particles

Handles global localization and tracking

Requires known map

Robot Localization (EKF/UKF):

Sensor fusion (IMU + Odometry + GPS)

Continuous pose estimation

No map required

Better for smooth tracking

⚙️ Setup and Installation
Step 1: Install Nav2 Packages

    #Install Nav2 core packages
    sudo apt-get install ros-humble-nav2-bringup
    sudo apt-get install ros-humble-nav2-common
    sudo apt-get install ros-humble-nav2-amcl
    sudo apt-get install ros-humble-nav2-behavior-tree
    sudo apt-get install ros-humble-nav2-behaviors
    sudo apt-get install ros-humble-nav2-bt-navigator
    sudo apt-get install ros-humble-nav2-controller
    sudo apt-get install ros-humble-nav2-core
    sudo apt-get install ros-humble-nav2-costmap-2d
    sudo apt-get install ros-humble-nav2-lifecycle-manager
    sudo apt-get install ros-humble-nav2-map-server
    sudo apt-get install ros-humble-nav2-navfn-planner
    sudo apt-get install ros-humble-nav2-planner
    sudo apt-get install ros-humble-nav2-regulated-pure-pursuit-controller
    sudo apt-get install ros-humble-nav2-rotation-shim-controller
    sudo apt-get install ros-humble-nav2-rviz-plugins
    sudo apt-get install ros-humble-nav2-smac-planner
    sudo apt-get install ros-humble-nav2-velocity-smoother
    sudo apt-get install ros-humble-nav2-voxel-grid

    #Install SLAM packages
    sudo apt-get install ros-humble-slam-toolbox
    sudo apt-get install ros-humble-slam-toolbox-msgs
    sudo apt-get install ros-humble-cartographer
    sudo apt-get install ros-humble-cartographer-ros

    #Install localization packages
    sudo apt-get install ros-humble-robot-localization
    sudo apt-get install ros-humble-tf2-* 
    sudo apt-get install ros-humble-imu-filter-madgwick

    #Install navigation tutorials
    sudo apt-get install ros-humble-navigation2
    sudo apt-get install ros-humble-nav2-minimal-tb3
    sudo apt-get install ros-humble-nav2-minimal-tb4

Step 2: Create Navigation Package

    cd ~/ros2_ws/src
    ros2 pkg create robot_navigation --build-type ament_python \
        --dependencies rclpy rclcpp nav2_msgs nav2_util nav2_amcl \
                     nav2_lifecycle_manager nav2_map_server \
                     nav2_planner nav2_controller nav2_costmap_2d \
                     nav2_navfn_planner nav2_smac_planner \
                     nav2_regulated_pure_pursuit_controller \
                     slam_toolbox robot_localization tf2_ros \
        --description "Week 6: ROS 2 Navigation Stack"

    cd robot_navigation
    mkdir -p robot_navigation/{maps,params,launch,config,behavior_trees}
    mkdir -p config/{nav2_params,navigation,localization}
    mkdir -p maps/{warehouse,office,maze,lab}
    mkdir -p launch/navigation
    mkdir -p behavior_trees/custom

🔧 Practical Exercises

Exercise 1: Basic Nav2 Setup with TurtleBot3

1.1 Install TurtleBot3 Simulation:

    #Install TurtleBot3 packages
    sudo apt-get install ros-humble-turtlebot3-gazebo
    sudo apt-get install ros-humble-turtlebot3-navigation2
    sudo apt-get install ros-humble-turtlebot3-cartographer
    sudo apt-get install ros-humble-turtlebot3-teleop

    #Set TurtleBot3 model
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    source ~/.bashrc
1.2 Test Navigation with TurtleBot3:

    #Terminal 1: Launch TurtleBot3 in Gazebo
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

    #Terminal 2: Launch Nav2 stack
    ros2 launch turtlebot3_navigation2 navigation2.launch.py \
        use_sim_time:=True \
        map:=/opt/ros/humble/share/turtlebot3_navigation2/map/turtlebot3_world.yaml

    #Terminal 3: Set initial pose
    ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
    header:
      frame_id: map
    pose:
      pose:
        position:
          x: -2.0
          y: -0.5
      covariance: [0.1, 0, 0, 0, 0, 0,
                   0, 0.1, 0, 0, 0, 0,
                   0, 0, 0.1, 0, 0, 0,
                   0, 0, 0, 0.1, 0, 0,
                   0, 0, 0, 0, 0.1, 0,
                   0, 0, 0, 0, 0, 0.1]"

    #Terminal 4: Send navigation goal
    ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "
    header:
      frame_id: map
    pose:
      position:
        x: 2.0
        y: 1.0
      orientation:
        w: 1.0"

    #Terminal 5: View navigation status
    ros2 topic echo /navigation_velocity_smoother/parameter_events
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
    pose:
      header:
        frame_id: map
      pose:
        position:
          x: 2.0
          y: 0.5
        orientation:
          w: 1.0"
1.3 Visualize Navigation in RViz:

    #Launch RViz with Nav2 configuration
    rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

    #Add displays:
    # - Map: Topic /map
    # - RobotModel: Description Topic /robot_description
    # - Path: Topic /plan
    # - Pose: Topic /amcl_pose
    # - LaserScan: Topic /scan
          
    #Launch RViz with Nav2 configuration
    rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

    #Add displays:
    # - Map: Topic /map
    # - RobotModel: Description Topic /robot_description
    # - Path: Topic /plan
    # - Pose: Topic /amcl_pose
    # - LaserScan: Topic /scan

Exercise 2: Create Custom Navigation Robot
2.1 Create Robot Description with Nav2 Configuration:

    Create urdf/navigation_robot.urdf.xacro:

    <?xml version="1.0"?>
    <robot name="navigation_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    
        <!-- Include base robot description -->
        <xacro:include filename="$(find gazebo_simulation)/urdf/simple_robot.urdf.xacro"/>
    
        <!-- Add navigation-specific components -->
    
        <!-- Additional IMU for better localization -->
        <link name="nav_imu_link">
            <visual>
                <geometry>
                    <box size="0.03 0.03 0.02"/>
                </geometry>
                <material name="blue"/>
            </visual>
        </link>
    
        <joint name="nav_imu_joint" type="fixed">
            <parent link="base_link"/>
            <child link="nav_imu_link"/>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
        </joint>
    
        <!-- GPS antenna (simulated) -->
        <link name="gps_antenna">
            <visual>
                <geometry>
                    <sphere radius="0.02"/>
                </geometry>
                <material name="red"/>
            </visual>
        </link>
    
        <joint name="gps_joint" type="fixed">
            <parent link="base_link"/>
            <child link="gps_antenna"/>
            <origin xyz="0.1 0 0.15"/>
        </joint>
    
        <!-- Gazebo plugins for navigation -->
        <gazebo>
            <!-- IMU plugin -->
            <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
                <ros>
                    <namespace>/robot</namespace>
                    <argument>imu:=imu</argument>
                </ros>
                <update_rate>50</update_rate>
                <topic_name>imu</topic_name>
                <frame_name>nav_imu_link</frame_name>
                <xyz_offset>0 0 0</xyz_offset>
                <rpy_offset>0 0 0</rpy_offset>
            </plugin>
        
            <!-- Differential drive plugin with odometry -->
            <plugin name="differential_drive_controller" 
                    filename="libgazebo_ros_diff_drive.so">
                <ros>
                    <namespace>/robot</namespace>
                </ros>
                <update_rate>50</update_rate>
                <left_joint>left_wheel_joint</left_joint>
                <right_joint>right_wheel_joint</right_joint>
                <wheel_separation>0.4</wheel_separation>
                <wheel_diameter>0.2</wheel_diameter>
                <max_wheel_torque>20</max_wheel_torque>
                <max_wheel_acceleration>1.0</max_wheel_acceleration>
                <command_topic>cmd_vel</command_topic>
                <odometry_topic>odom</odometry_topic>
                <odometry_frame>odom</odometry_frame>
                <robot_base_frame>base_link</robot_base_frame>
                <publish_odom>true</publish_odom>
                <publish_odom_tf>true</publish_odom_tf>
                <publish_wheel_tf>true</publish_wheel_tf>
            </plugin>
        </gazebo>
    
    </robot>

2.2 Nav2 Parameters Configuration:

Create config/nav2_params/navigation_params.yaml:

    # Navigation2 Parameters
    amcl:
      ros__parameters:
        use_sim_time: True
        alpha1: 0.2
        alpha2: 0.2
        alpha3: 0.2
        alpha4: 0.2
        alpha5: 0.2
        base_frame_id: "base_link"
        beam_skip_distance: 0.5
        beam_skip_error_threshold: 0.9
        beam_skip_threshold: 0.3
        do_beamskip: false
        global_frame_id: "map"
        lambda_short: 0.1
        laser_likelihood_max_dist: 2.0
        laser_max_range: 100.0
        laser_min_range: -1.0
        laser_model_type: "likelihood_field"
        max_beams: 60
        max_particles: 2000
        min_particles: 500
        odom_frame_id: "odom"
        pf_err: 0.05
        pf_z: 0.99
        recovery_alpha_fast: 0.0
        recovery_alpha_slow: 0.0
        resample_interval: 1
        robot_model_type: "differential"
        save_pose_rate: 0.5
        sigma_hit: 0.2
        tf_broadcast: true
        transform_tolerance: 1.0
        update_min_a: 0.2
        update_min_d: 0.25
        z_hit: 0.5
        z_max: 0.05
        z_rand: 0.5
        z_short: 0.05
        scan_topic: /robot/scan

    #Global Costmap Parameters
    global_costmap:
      global_costmap:
        ros__parameters:
          use_sim_time: True
          global_frame: map
          robot_base_frame: base_link
          update_frequency: 1.0
          publish_frequency: 1.0
          width: 20.0
          height: 20.0
          resolution: 0.05
          origin_x: -10.0
          origin_y: -10.0
          rolling_window: false
          always_send_full_costmap: true
      
          # Layers
          plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      
          static_layer:
            plugin: "nav2_costmap_2d::StaticLayer"
            map_subscribe_transient_local: True
            unknown_cost_value: 128
            lethal_cost_threshold: 100
      
          obstacle_layer:
            plugin: "nav2_costmap_2d::ObstacleLayer"
            enabled: True
            observation_sources: scan
            scan:
              topic: /robot/scan
              max_obstacle_height: 2.0
              clearing: True
              marking: True
              data_type: "LaserScan"
              raytrace_max_range: 3.0
              raytrace_min_range: 0.0
              obstacle_max_range: 2.5
              obstacle_min_range: 0.0
      
          inflation_layer:
            plugin: "nav2_costmap_2d::InflationLayer"
            cost_scaling_factor: 3.0
            inflation_radius: 0.55
      
          # Footprint
          footprint: "[[-0.25, -0.2], [-0.25, 0.2], [0.25, 0.2], [0.25, -0.2]]"
          footprint_padding: 0.01
      
          # Filters
          keepout_filter:
            plugin: "nav2_costmap_2d::KeepoutFilter"
            enabled: False
      
          speed_filter:
            plugin: "nav2_costmap_2d::SpeedFilter"
            enabled: False

    # Local Costmap Parameters
    local_costmap:
      local_costmap:
        ros__parameters:
          use_sim_time: True
          global_frame: odom
          robot_base_frame: base_link
          update_frequency: 5.0
          publish_frequency: 2.0
          width: 3.0
          height: 3.0
          resolution: 0.05
          origin_x: -1.5
          origin_y: -1.5
          rolling_window: true
      
          plugins: ["voxel_layer", "inflation_layer"]
      
          voxel_layer:
            plugin: "nav2_costmap_2d::VoxelLayer"
            enabled: True
            publish_voxel: False
            origin_z: 0.0
            z_resolution: 0.2
            z_voxels: 2
            max_obstacle_height: 2.0
            mark_threshold: 0
            observation_sources: scan
            scan:
              topic: /robot/scan
              max_obstacle_height: 2.0
              clearing: True
              marking: True
              data_type: "LaserScan"
              raytrace_max_range: 3.0
              raytrace_min_range: 0.0
              obstacle_max_range: 2.5
              obstacle_min_range: 0.0
      
          inflation_layer:
            plugin: "nav2_costmap_2d::InflationLayer"
            cost_scaling_factor: 3.0
            inflation_radius: 0.55
      
          footprint: "[[-0.25, -0.2], [-0.25, 0.2], [0.25, 0.2], [0.25, -0.2]]"
          footprint_padding: 0.01
          transform_tolerance: 0.2

    # Global Planner
    planner_server:
      ros__parameters:
        use_sim_time: True
        planner_plugins: ['GridBased']
    
        GridBased:
          plugin: "nav2_navfn_planner/NavfnPlanner"
          tolerance: 0.5
          use_astar: true
          allow_unknown: true
    
        # Alternative: Smac Planner for 2D
        SmacPlanner2D:
          plugin: "nav2_smac_planner/SmacPlanner2D"
          tolerance: 0.25
          downsample_costmap: true
          downsampling_factor: 1
          allow_unknown: false
          max_iterations: 1000000
          max_on_approach_iterations: 1000
          terminal_checking_interval: 100
          use_final_approach_orientation: false
          motion_model_for_search: "MOORE"

    # Local Controller
    controller_server:
      ros__parameters:
        use_sim_time: True
        controller_frequency: 20.0
        min_x_velocity_threshold: 0.001
        min_y_velocity_threshold: 0.5
        min_theta_velocity_threshold: 0.001
        progress_checker_plugin: "progress_checker"
        goal_checker_plugin: "general_goal_checker"
        controller_plugins: ["FollowPath"]
    
        # Progress checker
        progress_checker:
          plugin: "nav2_controller::SimpleProgressChecker"
          required_movement_radius: 0.5
          movement_time_allowance: 10.0
    
        # Goal checker
        general_goal_checker:
          plugin: "nav2_controller::SimpleGoalChecker"
          xy_goal_tolerance: 0.25
          yaw_goal_tolerance: 0.25
          stateful: True
    
        # Controller
        FollowPath:
          plugin: "nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController"
          desired_linear_vel: 0.5
          lookahead_dist: 0.6
          min_lookahead_dist: 0.3
          max_lookahead_dist: 0.9
          lookahead_time: 1.5
          rotate_to_heading_min_angle: 0.785
          rotate_to_heading_angular_vel: 1.8
          transform_tolerance: 0.2
          use_velocity_scaled_lookahead_dist: true
          min_approach_linear_velocity: 0.05
          approach_velocity_scaling_dist: 0.6
          max_allowed_time_to_collision_up_to_carrot: 1.0
          regulated_linear_scaling_min_radius: 0.9
          regulated_linear_scaling_min_speed: 0.25
          use_fixed_curvature_lookahead: false
          use_rotate_to_heading: true
          use_interpolation: true
          use_cost_regulated_linear_velocity_scaling: true
          cost_scaling_dist: 1.0
          cost_scaling_gain: 1.0
          inflation_cost_scaling_factor: 3.0

    # Behavior Server (Recovery Behaviors)
    behavior_server:
      ros__parameters:
        use_sim_time: True
        behavior_plugins: ["spin", "backup", "wait"]
    
        spin:
          plugin: "nav2_behaviors/Spin"
          simulate_ahead_time: 2.0
          max_rotational_vel: 1.0
          min_rotational_vel: 0.2
          rotational_acc_lim: 3.2
    
        backup:
          plugin: "nav2_behaviors/BackUp"
          simulate_ahead_time: 2.0
          max_linear_vel: 0.5
          min_linear_vel: 0.1
          linear_acc_lim: 2.5
    
        wait:
          plugin: "nav2_behaviors/Wait"
          wait_duration: 5.0
    
        local_costmap_topic: local_costmap/costmap_raw
        local_footprint_topic: local_costmap/footprint
        global_costmap_topic: global_costmap/costmap_raw
        global_footprint_topic: global_costmap/footprint
        cycle_frequency: 10.0

    # Velocity Smoother
    velocity_smoother:
      ros__parameters:
        use_sim_time: True
        smoothing_frequency: 20.0
        scale_velocities: False
        feedback: "OPEN_LOOP"
    
        max_velocity: [0.5, 0.0, 1.8]
        min_velocity: [-0.3, 0.0, -1.8]
    
        max_accel: [2.5, 0.0, 3.2]
        max_decel: [2.5, 0.0, 3.2]
    
        odom_topic: /robot/odom

    # Waypoint Follower
    waypoint_follower:
      ros__parameters:
        use_sim_time: True
        loop_rate: 20
        stop_on_failure: true
        waypoint_task_executor_plugin: "wait_at_waypoint"
        wait_at_waypoint:
          plugin: "nav2_waypoint_follower::WaitAtWaypoint"
          enabled: True
          waypoint_pause_duration: 2.0

    # BT Navigator
    bt_navigator:
      ros__parameters:
        use_sim_time: True
        global_frame: map
        robot_base_frame: base_link
        odom_topic: /robot/odom
        bt_loop_duration: 10
        default_server_timeout: 20
        enable_groot_monitoring: True
        groot_zmq_publisher_port: 1666
        groot_zmq_server_port: 1667
        plugin_lib_names:
        - nav2_compute_path_to_pose_action_bt_node
        - nav2_follow_path_action_bt_node
        - nav2_back_up_action_bt_node
        - nav2_spin_action_bt_node
        - nav2_wait_action_bt_node
        - nav2_clear_costmap_service_bt_node
        - nav2_is_stuck_condition_bt_node
        - nav2_goal_reached_condition_bt_node
        - nav2_initial_pose_received_condition_bt_node
        - nav2_reinitialize_global_localization_service_bt_node
        - nav2_rate_controller_bt_node
        - nav2_distance_controller_bt_node
        - nav2_speed_controller_bt_node
        - nav2_truncate_path_action_bt_node
        - nav2_goal_updater_node_bt_node
        - nav2_recovery_node_bt_node
        - nav2_pipeline_sequence_bt_node
        - nav2_round_robin_node_bt_node
        - nav2_transform_available_condition_bt_node
        - nav2_time_expired_condition_bt_node
        - nav2_path_expiring_timer_condition
        - nav2_distance_traveled_condition_bt_node

    # Lifecycle Manager
    nav2_lifecycle_manager:
      ros__parameters:
        use_sim_time: True
        autostart: True
        node_names:
          - amcl
          - map_server
          - planner_server
          - controller_server
          - behavior_server
          - bt_navigator
          - velocity_smoother
          - waypoint_follower

2.3 Robot Localization Configuration (EKF):

Create config/localization/ekf.yaml:

    #Extended Kalman Filter for sensor fusion
    ekf_filter_node:
      ros__parameters:
        use_sim_time: True
    
    # Frequency
        frequency: 30.0
    
    # Sensor timeout
        sensor_timeout: 0.1
    
    # 2D mode
        two_d_mode: True
    
    # Publish TF
        publish_tf: True
        map_frame: map
        odom_frame: odom
        base_link_frame: base_link
        world_frame: odom
    
    # Odometry sensor
        odom0: /robot/odom
        odom0_config: [true, true, true,
                       false, false, true,
                       false, false, false,
                       false, false, false,
                       false, false, false]
        odom0_queue_size: 10
        odom0_nodelay: true
        odom0_differential: false
        odom0_relative: false
    
    # IMU sensor
        imu0: /robot/imu
        imu0_config: [false, false, false,
                      true, true, true,
                      false, false, false,
                      false, false, true,
                      true, true, true]
        imu0_queue_size: 10
        imu0_nodelay: true
        imu0_differential: false
        imu0_relative: true
        imu0_remove_gravitational_acceleration: true
    
        # Process noise covariance
        process_noise_covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02]
2.4 Nav2 Launch File:

Create launch/navigation/navigation_stack.launch.py:

    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
    from launch.conditions import IfCondition, UnlessCondition
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    import os

    def generate_launch_description():
        # Package directories
        pkg_navigation = FindPackageShare('robot_navigation')
        pkg_gazebo_sim = FindPackageShare('gazebo_simulation')
    
        # Launch configurations
        use_sim_time = LaunchConfiguration('use_sim_time', default='True')
        map_yaml_file = LaunchConfiguration('map', default='')
        autostart = LaunchConfiguration('autostart', default='True')
        params_file = LaunchConfiguration('params_file', default='')
    
        # Declare launch arguments
        declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        )
    
        declare_map_yaml_cmd = DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                pkg_navigation, 'maps', 'warehouse', 'warehouse.yaml'
            ]),
            description='Full path to map yaml file'
        )
    
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                pkg_navigation, 'config', 'nav2_params', 'navigation_params.yaml'
            ]),
            description='Full path to the ROS2 parameters file to use'
        )
    
        declare_autostart_cmd = DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Automatically startup the nav2 stack'
        )
    
        # Map server
        map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'yaml_filename': map_yaml_file}]
        )
    
        # AMCL
        amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        )
    
        # Controller server
        controller_server_node = Node(
            package='nav2_controller_server',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        )
    
        # Planner server
        planner_server_node = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        )
    
        # Behavior server
        behavior_server_node = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        )
    
        # BT Navigator
        bt_navigator_node = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        )
    
        # Waypoint follower
        waypoint_follower_node = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        )
    
        # Velocity smoother
        velocity_smoother_node = Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        )
    
        # Lifecycle manager
        lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': autostart,
                         'node_names': [
                             'map_server',
                             'amcl',
                             'planner_server',
                             'controller_server',
                             'behavior_server',
                             'bt_navigator',
                             'waypoint_follower',
                             'velocity_smoother'
                         ]}]
        )
    
        # Robot localization (EKF)
        ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                pkg_navigation, 'config', 'localization', 'ekf.yaml'
            ]), {'use_sim_time': use_sim_time}]
        )
    
        # RViz with Nav2 plugins
        rviz_config_file = PathJoinSubstitution([
            pkg_navigation, 'config', 'navigation.rviz'
        ])
    
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_rviz', default='True'))
        )
    
        # Static TF for map->odom (will be overwritten by AMCL)
        static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    
        return LaunchDescription([
            declare_use_sim_time_cmd,
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_autostart_cmd,
        
            map_server_node,
            amcl_node,
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            velocity_smoother_node,
            lifecycle_manager_node,
            ekf_node,
            rviz_node,
            static_tf_node
        ])
    
2.5 Integration with Gazebo:
Create launch/navigation/navigation_gazebo.launch.py:

    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription, TimerAction
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare

    def generate_launch_description():
        return LaunchDescription([
            # Start Gazebo with world
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('gazebo_simulation'),
                    '/launch/spawn_robot.launch.py'
                ]),
                launch_arguments={
                    'world': 'maze.sdf',
                    'use_sim_time': 'True'
                }.items()
            ),
        
            # Wait for robot to spawn
            TimerAction(
                period=5.0,
                actions=[
                    # Start navigation stack
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            FindPackageShare('robot_navigation'),
                            '/launch/navigation/navigation_stack.launch.py'
                        ]),
                        launch_arguments={
                            'use_sim_time': 'True',
                            'map': FindPackageShare('robot_navigation') + '/maps/maze/maze.yaml',
                            'autostart': 'True'
                        }.items()
                    )
                ]
            ),
        
            # Start teleop for manual control (optional)
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package='teleop_twist_keyboard',
                        executable='teleop_twist_keyboard',
                        name='teleop',
                        prefix='xterm -e',
                        remappings=[('/cmd_vel', '/robot/cmd_vel')]
                    )
                ]
            )
        ])


Exercise 3: SLAM - Simultaneous Localization and Mapping
