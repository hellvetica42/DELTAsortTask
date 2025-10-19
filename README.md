## Running project
Build docker container with

```docker build -t cyclone-dearpygui . ```

Then run it with

```./run_docker.sh```

and attach another terminal with

```./attach_docker.sh```

Go to ```/app/src/``` with both terminals.
Run ```python3 GUI.py``` on one terminal and ```python3 main.py``` in the other terminal.

## The task
This project is a dummy example of a pick and place system with a conveyor belt. 
The GUI you see is a visual of all the key points in the system. 
- The red points are detections in the "camera" (in this case just a black texture). 
- The green points are tracking points that predict future positions of objects that leave the camera frame
- The yellow diamond is the current position of the robot
- The green cross is the target position of the robot
- The blue cross is a debug point to show the robot's next move

Your task is to design an algorithm to:
1. Match the timing of the robot and object movement so the end effector reaches the object position.
2. Pick as many objects as possible without increasing travel speed. By "pick" we mean reach the object's position and then move the robot to the side.

Consider that the algorithm you write will be applied to a real delta robot, so the decisions it makes have to be precise and absolute. (meaning no jittery movement).
Because of this __timing is key__. 

Your algorithm should be run in Decision.py in the ```update_decision``` function.
However, we encourage you to dig a bit through this example to see how the robot and other interfaces work.
You should start with ```DummyDelta.py``` and ```CameraCalibration/ConveyorTracker.py```.


