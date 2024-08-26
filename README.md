# GestureManibot Project
This is a self-learning project that focuses on using the Moveit2 and ROS humble to develop a pipeline that uses simple hand gestures recognised through webcam to enable a manipulator robot to operate on objects nearby.
The main concept used here is the Moveit Task constructor which helps in developing tasks in form of stages. 

This project is an experiment to understand the concepts of skills that can enable a mobile manipulator identify relevant blocks on the field and operate by optimal decision making.

More information can be explored in the link "https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html".

## Description of the pipeline
1. Using laptop webcam, a ros package called py_handgesture extracts the objective for the manipulator robot. This handgesture uses trained models from mediapipe.
2. Next, these gesture classes are published to a rostopic which provides task information for the manipulator