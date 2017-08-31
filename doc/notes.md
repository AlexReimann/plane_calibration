Random scribbles
================

Hough transform for planes R3 in depth images
------------------------------------------

* Planes have only point + 2 degree of freedoms
    * Rotating around the normal vector is invariant -> 2 DOF
    * Point "fixes" distance?

Basically if you have a plane going through a point, this plane has only 2 DOF (+ invariant).

-> Should be able to parameterize with two parameters in a certain space?

The two parameters should be same for all point-plane combinations <- hard part

Plane parameterization are typically:
- ax + by + cz + d = 0
- Normal vector + point on vector
- Point + two vectors
- 3 points


Use 2 points on axis? 
- Have restrictions?
