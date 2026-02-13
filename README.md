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

Component	          Package	                          Purpose

Map Server	        nav2_map_server	                  Provides static map data

AMCL	              nav2_amcl	                        Adaptive Monte Carlo Localization

Planner Server	    nav2_planner	                    Global path planning

Controller Server	  nav2_controller	                  Local trajectory planning

Behavior Server	    nav2_behaviors	                  Recovery behaviors

Costmap 2D	        nav2_costmap_2d	                  Obstacle representation

Lifecycle           Manager	nav2_lifecycle_manager	  Node lifecycle management
