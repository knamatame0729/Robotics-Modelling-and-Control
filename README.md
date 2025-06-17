# Robotics-Modelling-and-Control 

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

### Representation of a Vector
Let **p** be a vector expressed in the reference frame. When we apply a rotation matrix R** to **p**, the result is a new vector *R***p**, which is the rotated version of **p**:  

<div align="center">

**p'** = *R***p**
  
</div>


### Composition of Rotation Matrices
Let O-x₀y₀z₀, O-x₁y₁z₁, O-x₂y₂z₂ be three frames with common origin O.  

Let **p₂** be the coordinates of a vector **p** in Frame 2. Then, the transformations between frames are given by:

<div align="center">

**p₁** = R12 **p₂**  
**p₀** = R01 **p₁**  

</div>

By substituting,

<div align="center">

**p₀** = R01 R12 **p₂**  
**p₀** = R02 **p₂**

</div>

 

- If each rotation is defined with respect to the current(rotating) frame, apply them using post-multiplication in the other they occur:  
<div align="center">

*R(final)* = *R₁R₂R₃ ...*

</div>



- If each rotation is defined with respect to a fixed(initial) frame, apply them using pre-multiplication in reverse order: 
<div align="center">

*R(final)* = *Rₙ ... R₂R₁*

</div> 

## Kinematics
- **Forward Kinematics (FK)** - Computing an end-effector pose from joint angles, visualising reachable workspace, and implementing an efficient single-pass FK algorithm for arbitrary chains.
- **Differential Kinematics** - Deriving the Jacobian matirix, interpreting its geometric meaning, plotting the manipulability ellipse, and detecting singularities via rank/condition tests.
- **Inverse Kinematics (IK)** - Solving the pose-to-joint problem through both analytic and numerical techniques.

## Position of a Rigid Body
A rigid body is completely described in space by its position and orientation with respect to a reference frame. 

## Workspace visualisation (planar 2‑R)

[workspace_visualize.py](https://github.com/knamatame0729/Robotics-Modelling-and-Control/blob/main/workspace_visualize.py)

![alt text](media/reachable_ws.png)  

![alt text](media/pose.png)  

![alt text](media/rechable_ws_orientation.png)  

## Single‑Pass FK Algorithm implementation
The sigle-pass FK algorithm computes every link's world tranform in one linear sweep - no recursion, no call-stack growth.  

- Efficiency - Linear time, cache-friendly loop.
- Generality - Handles chains, trees, even floating bases (set parent = -1 and provide its pose).
- Simplicity - Mirrors physical assembly: base → tip.

## Differential Kinematics
The forward kinematics map  

<div align="center">

*f* : *Q* → *SE(3)*, *x = f(q)*

</div>

is defferntiable almost everywhere. Linearizing it around a configuration **q** yields the core relation:  

<div align="center">

*x˙=J(q)q˙*

</div>

where *J(q) is the Jacobian matrix.

## Jacobian properties: rank, manipulability, ellipse

## Error Propagation Ellipse  
Measurements or commands in joint space are never perfect. If the joint vector q carries uncertainty, that error propagates through the robot's kinematics and affects the position of the end effector in task space.  

[error_propagation_ellipse.py](https://github.com/knamatame0729/Robotics-Modelling-and-Control/blob/main/error_propagation_ellipse.py)  

- Result  (Joint angle uncertainty 0.05 [rad])  
![Image](https://github.com/user-attachments/assets/3c981419-f66c-48eb-8a93-6f5ed2b2e6de)
