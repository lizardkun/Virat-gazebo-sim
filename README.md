# Virat-gazebo-sim
Gazebo simulation of a pothole detecting remote controlled robot

### Loading the urdf on a grassy plane
![Screenshot from 2022-08-07 17-40-00](https://user-images.githubusercontent.com/94188928/183289865-8a7fca95-02bd-47cd-a774-04bad8b574c5.png)
### Moving it around with a simple tele-op code
[Screencast from 08-07-2022 05:56:50 PM.webm](https://user-images.githubusercontent.com/94188928/183290517-67258ad8-3563-4ba7-bb8c-d96e3bcc0d86.webm)
### See what Virat Jr sees on ROS Visualizer
![Screenshot from 2022-08-07 17-47-25](https://user-images.githubusercontent.com/94188928/183290153-f2754788-38ec-48f2-85c5-1e98f9376c52.png)
### Virat Jr uses openCV to detect potholes:
![Screenshot from 2022-08-07 17-35-39](https://user-images.githubusercontent.com/94188928/183289708-b3437405-b9f1-4af1-a9fe-967dc55a6cd3.png)
### The detected potholes are mapped on RViz with pointclouds
The pixels generated from the contours are converted to ground frame distances wrt to the camera using inverse perspective mapping (this formula is used):
![image](https://user-images.githubusercontent.com/94188928/183304762-854ea2c4-e679-457e-8b06-b730e416202c.png)
The rviz map we get:

![image](https://user-images.githubusercontent.com/94188928/183304239-18e72462-9dc9-4416-a63d-06dcf48656c2.png)

### Altogether:

![image](https://user-images.githubusercontent.com/94188928/183304307-d6dc0cff-0dd3-4614-94ca-3b278170c752.png)
