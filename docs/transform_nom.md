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
<img src="../recursos/imgs/trsf_nom_1.jpeg" alt="Ejercico 1" width="420">

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


!!! tip "Consejo"
    Mantén este resumen corto (máx. 5 líneas). Lo demás va en secciones específicas.

---

## 2) Objetivos

- **General:** _Qué se pretende lograr en términos amplios._
- **Específicos:**
  - _OE1…_
  - _OE2…_
  - _OE3…_

## 3) Alcance y Exclusiones

- **Incluye:** _Qué funcionalidades/entregables sí están en el proyecto._
- **No incluye:** _Qué queda fuera para evitar malentendidos._

---

## 4) Requisitos

**Software**
- _SO compatible (Windows/Linux/macOS)_
- _Python 3.x / Node 18+ / Arduino IDE / etc._
- _Dependencias (p. ej., pip/requirements, npm packages)_

**Hardware (si aplica)**
- _MCU / Sensores / Actuadores / Fuente de poder_
- _Herramientas (multímetro, cautín, etc.)_

**Conocimientos previos**
- _Programación básica en X_
- _Electrónica básica_
- _Git/GitHub_

---

## 5) Instalación