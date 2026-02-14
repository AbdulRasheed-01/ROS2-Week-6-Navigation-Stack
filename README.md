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

          
