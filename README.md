# Robotics-Modelling-and-Control

## Kinematics
- **Forward Kinematics (FK)** - Computing an end-effector pose from joint angles, visualising reachable workspace, and implementing an efficient single-pass FK algorithm for arbitrary chains.
- **Differential Kinematics** - Deriving the Jacobian matirix, interpreting its geometric meaning, plotting the manipulability ellipse, and detecting singularities via rank/condition tests.
- **Inverse Kinematics (IK)** - Solving the pose-to-joint problem through both analytic and numerical techniques.

## Position of a Rigid Body
A rigid body is completely described in space by its position and orientation with respect to a reference frame.  

- **Reference Frame: F = O-xyz**  
Origin: O
Unit vectors: x, y, z(along each coordinate axis)  
Suppose there is a fixed point O' on the rigid body. Its position with respect to the frame F is given by a position vector $\mathbf{o}' \in \mathbb{R}^3$.  

- **Representation of the Position Vector**  
The vector o' can be expressed as  
![alt text](media/image.png) 

- **What is a Bound Vector ?**  
A bound vector is a vector that not only has magnitude and direction, but also a fixed point of application and acts a specific line in space.

## Orientation of a Rigid Body
The orientation of a rigid body is described by attaching a new orthonormal frame  O′-x′y′z′  to the body and expressing its axes with respect to the reference frame  O-xyz .

Each unit vector  x′,y′,z′  of the body frame is expressed in terms of the reference frame as:  
![alt text](media/image-1.png)

## Rotation Matrix
A rotation matrix provides a way to describe the orientation of a rigid body in three-dimensional space.
It is formed by arranging the unit vectors of the rotated coordinate frame into a 3×3 matrix R  
![alt text](media/rotation_matrix.png)

Rotations in 3D space can be described using elementary rotation matrices, each representing a rotation about one of the main coordinate axes of a fixed reference fram O-xyz. A rotation is considered positive if it is counter-clockwise when viewed along the direction of the axis.  

- A rotation about teh x-axis by angle γ keeps the x-axis fixed and rotates the y- and z-aixs.  
![alt text](media/x_axis.png)

- A rotation about the y-axis by angle β fixes the y-axis, rotating teh x- and z-axis.  
![alt text](media/y_axis.png)

- A rotation about the z-axis by angle α fixes the z-axis, rotating the x- and y-axis in the xy-plane.  
![alt text](media/z_axis.png) 

## Workspace visualisation (planar 2‑R)

## Single‑Pass FK Algorithm implementation

## Jacobian properties: rank, manipulability, ellipse

## Error Propagation Ellipse  
Measurements or commands in joint space are never perfect. If the joint vector q carries uncertainty, that error propagates through the robot's kinematics and affects the position of the end effector in task space.  

[error_propagation_ellipse.py](https://github.com/knamatame0729/Robotics-Modelling-and-Control/blob/main/error_propagation_ellipse.py)  

- Result  (Joint angle uncertainty 0.05 [rad])  
![Image](https://github.com/user-attachments/assets/3c981419-f66c-48eb-8a93-6f5ed2b2e6de)
