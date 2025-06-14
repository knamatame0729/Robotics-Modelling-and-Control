# Robotics-Modelling-and-Control

## Kinematics
- **Forward Kinematics (FK)** - Computing an end-effector pose from joint angles, visualising reachable workspace, and implementing an efficient single-pass FK algorithm for arbitrary chains.
- **Differential Kinematics** - Deriving the Jacobian matirix, interpreting its geometric meaning, plotting the manipulability ellipse, and detecting singularities via rank/condition tests.
- **Inverse Kinematics (IK)** - Solving the pose-to-joint problem through both analytic and numerical techniques.

## Position of a Rigid Body

## Orientation of a Rigid Body

## Rotation Matrix

## Workspace visualisation (planar 2‑R)

## Single‑Pass FK Algorithm implementation

## Jacobian properties: rank, manipulability, ellipse

## Error Propagation Ellipse  
Measurements or commands in joint space are never perfect. If the joint vector q carries uncertainty, that error propagates through the robot's kinematics and affects the position of the end effector in task space.  

[error_propagation_ellipse.py](https://github.com/knamatame0729/Robotics-Modelling-and-Control/blob/main/error_propagation_ellipse.py)  

- Result  
![Image](https://github.com/user-attachments/assets/3c981419-f66c-48eb-8a93-6f5ed2b2e6de)