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

