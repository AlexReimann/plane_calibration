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


Restricted max angle deviation

Hough:
* Plane with distance + 2 angles
* Check amount of points fitting for each plane (voting)

--

* Angles are limited
* Thus point deviation from plane is limited 
* Max deviation for each pixel should be linear to plane distance d (or even fixed?).
* Z is independent from camera parameters

* If hough planes are parallel to viewing (fixed d), only angles variable
* The planes each pixel / point fits to is a fixed d range (because plane rotation is limited)
* So each pixel votes for the planes in certain d range
* ...?
* profit
