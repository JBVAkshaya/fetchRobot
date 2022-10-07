### Enable the Fetch Robot to participate in a Scavenger hunt
- Enabled the Fetch Robot to map and localize in real environment

### Usage

After cloning and building the fetch robot repository https://github.com/fetchrobotics/fetch_ros clone this repository in ```catkin_ws/src``` where all the other dependencies exits and build catkin workspace.

Launch the launch file:

```roslaunch fetch_world_alan.launch```

Run the script:

### Submodules

- Identified the object

  - Object Detection
  
  
  <img width="396" alt="image" src="https://user-images.githubusercontent.com/76990931/194443831-5e742a95-9378-4786-bb58-84edf9b1d92f.png">
  
  
  - Pose Estimation
  
  
  <img width="196" alt="image" src="https://user-images.githubusercontent.com/76990931/194443902-c5c4fbab-be5e-4969-935e-62ec65340447.png">
     
     
- Plan Path
  - RRTConnect is used to plan the mobile base path
-Arm Manipulation and Object Grasping
  - Collision aware inverse kinematics is used using the move-it package so as to avoid collision when planning arm movement

### Glimpse

Navigating

<img width="343" alt="image" src="https://user-images.githubusercontent.com/76990931/194444385-213b7b58-cc46-4e18-a1d4-306745390271.png">

Grasping

<img width="171" alt="image" src="https://user-images.githubusercontent.com/76990931/194444459-9c1c6d99-4cb5-4a31-95df-698c689e4be4.png">

Yay!!

<img width="343" alt="image" src="https://user-images.githubusercontent.com/76990931/194444541-a74feb83-9f5b-42cc-8ccb-56b680719a33.png">
