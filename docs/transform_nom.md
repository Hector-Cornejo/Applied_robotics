# Transform Nomenclature

> Activity:
>Write the correct nomenclature for the frame transform, in the following problems.

---
## Important concepts
-**What is the right hand rule and how does it help in this tipe of problems?**  
The right hand rule is used to define the orientaion of the Cartesian coordinate systems in three dimensions. In a right-handed coordinate system, the directions of the X, Y, and Z axes are related by the right hand: the **index finger** points along the X-axis, the **middle finger** points along the Y-axis,then the **thumb**, extended perpendicular to both, points in the direction of the Z-axis.
The right-hand rule is also a convention used to determine the direction of rotation and to define whether a rotation is positive or negative. To apply it, point **the thumb of your right hand in the direction of the positive axis of rotation**. The direction in which your fingers curl indicates the positive direction of rotation around that axis.

<img src="../recursos/imgs/right_hand.png" alt="axis" width="420">

## 1)
<img src="../recursos/imgs/trsf_nom_1.jpeg" alt="Ejercico 1" width="550">

The first step to solve this problem is to get the matrixes of each rotation of the stablished vector, it is important to solve it following the chronological order in which the problem states the rotations happened, in this case, first the rotation about the Y axis and then the rotation about the X axis.

-**How can rotation be visualized?**  
To better visualize rotational transformations, it is helpful to represent the vector within a Cartesian coordinate system. A rotation about a given axis leaves the component aligned with that axis unchanged, while the remaining components are transformed. For example, a rotation about the Y-axis affects only the X and Z components of the vector, as the rotation takes place in the plane perpendicular to the Y-axis. This behavior is reflected in the corresponding rotation matrix, where the Y component remains constant.

In the images below, in the drawn cartesian planes, it can be observed the original position vectors, represented by **X_1 and Y_1**, as well as the positions of said vectors after the rotation, represented by **X_2 and Y_2**. Since the problem also has the angle of rotation in both cases, we can use cos and sin relations to find the new coordinates, resulting in the matrixes of each corresponding rotation.

<img src="../recursos/imgs/rotation_y.png" alt="Rotation Y" width="420">

<img src="../recursos/imgs/rotation_x.png" alt="Rotation X" width="420">

This problem describes a vector experiencing two rotations, so after obtaining the matrix for each rotation we need to multiply said matrixes to obtain the rotation matrix **R** that performs the rotations in the indicated order.
It is important to remember that matrix multiplication is not commutative so the order in which they are multiplied matters. In the case of rotation matrixes, we order the multiplication from the latest matrix to the first one, as we can see in the following image:

<img src="../recursos/imgs/matrix_mul.png" alt="matrix multiplication" width="420">

In order to multiply two matrices, the number of columns of the first matrix must be equal to the number of rows of the second matrix. Each element of the resulting matrix is obtained by taking the dot product of a row from the first matrix with a column from the second matrix.

<img src="../recursos/imgs/matrix_res.png" alt="multiplication result" width="420">

Finally we just need to simplify our result and we get the rotation matrix **R** whit all its axis components:

<img src="../recursos/imgs/matrix_sim.png" alt="rotation matrix R" width="420">
---

## 2) 
<img src="../recursos/img/trsf_nom_2.png" alt="Ejercico 2" width="550">

In this problem it can be observed that the reference frame experiences two different kinds of movement, a rotation and then translation. Same as the last problem, the movement of the reference frame must be analyzed in order the problem to present them, in this case, first the rotation of the reference frame B with respect to the reference frame A, then the translation form B to A.

<img src="../recursos/imgs/plns_mmnt.jpeg" alt="planes_movement" width="420"

For the rotation of reference frame B the problem is asking to make said rotation with respect to reference frame A about X_A by 30. This means that we are going to use the right-hand rule in A in order to follow that movement when B is rotated. Like so:

<img src="../recursos/imgs/B_rot_A.jpeg" alt="Rotation of B with respect to A" width="420"

Notice that since the problem asks to make the rotation with respect to reference frame A about X_A, in the perspective of reference frame B the rotation is negative as such, the rotation angle in the view point of reference frame B is -30 degrees.  
By making the visualization of the rotation in reference frame B, its matrix can be calculated:

<img src="../recursos/imgs/mtx_rot_B.png" alt="The matrix of the rotation on B" width="420"

The next step would be to calculate the translation of B from A but in this case the problem already establishes it, but since its a 3D space the point is represented as such:

<img src="../recursos/imgs/trns_vect.jpeg" alt="Translation of B from A" width="420"

Now the problem is asking the formulation of a homogeneous transform matrix. Which is the combination of both rotation and translation in a single transformation. This results in a 4x4 matrix containing the matrix of the rotation and the translation vector. For this particular problem it looks like this:

<img src="../recursos/imgs/transform_matrix.jpeg" alt="Transformation matrix" width="420"