# Forward Kinematics

> Get the matrix and DH placement of the frames, for the following robots

---

## Important concepts
**What are forward Kinematics?**  
It refers to the study of motion, without considering forces, of a robot to calculate its final position and orientation of its end-effector, which refers to its last joint where a tool may be attached.

**How to obtain the forward kinematics of a robot?**
1) Step 1: Establish the Z axis of each joint.  
This depends mainly on the type of joint, the most used are the prismatic joint and the revolute joint. Those are the ones used in the following cases, therefore I’m only focusing on them.

<img src="../recursos/imgs/" alt="Prismatic and revolut joints" width="600">

- For a **Prismatic joint**, the Z axis is oriented along the joint linear displacement. This means that the Z axis will follow along in the direction between the **Link** of the interested **joint i-1** unto the next **joint i.**
- For a **revolute joint**, the Z axis is oriented along the angle of rotation. This means that the Z axis is the line in which the revolute joint turns about.

<img src="../recursos/imgs/" alt="Example of Z axis on prismatic and revolute joints." width="600">

2) Step 2: Establish the origins of each joint.  
Take the following sistem as an example:

<img src="../recursos/imgs/" alt="Example of forward kinematcs exercise." width="600">

As shown in the previous example, the first step is to define the **Z-axis of each joint**, which is chosen according to the type of joint and the direction of its motion. The origin of frame i is placed at the intersection of the Z-axis of the previous joint (Z _i-1_) and the Z-axis of the current joint (Z _i_).  

For the first joint, the origin can be placed arbitrarily, since there is no previous Z-axis to intersect with. However, **it is recommended to place the origin at the base of the joint**, as this simplifies the model and avoids the need to introduce additional measurements to account for offsets. If at any point the axes Z _i-1_ and Z _i_ do not intersect, the origin may be placed anywhere along the Z-axis of the current joint, considering it as an infinite line.

<img src="../recursos/imgs/" alt="Example of joint origins" width="600">

3) Step 3: Establish the X axis for each joint.  
The **X-axis** is placed orthogonally to both Z i-1 and Z i. This means that the X-axis forms a 90-degree angle with each of these Z-axes.

<img src="../recursos/imgs/" alt="Example of X axis on each joint" width="600">

4) Step 4: Establish the Y axis for each joint.
To obtain the **transformation matrix** for each joint, the Y-axis is not explicitly used. However, it is still good practice to define it in order to complete the reference frame of each joint. The Y-axis is defined according to the right-hand rule, so once the X- and Z-axes are established, the Y-axis is automatically determined.

<img src="../recursos/imgs/" alt="Right-Hand rule" width="600">

<img src="../recursos/imgs/" alt="Example of Y axis on each joint" width="600">

4) Step 5: The Denavit-Hartenberg Convention.  
A commonly used convention to select frames of reference in robotics is the Denavit-Hartenberg Convention. In this convention each homogeneous transformation matrix can be represented as the product of four basic transformations.

T_i = R_z(\theta_i) t_z(d_i) t_x(a_i) R_x(\alpha_i)