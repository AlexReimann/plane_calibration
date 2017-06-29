# plane_calibration
[ROS](http://www.ros.org/) package for robust and fast on the fly 3d sensor xy-orientation adjustment using the ground floor plane.
Bascially fits a plane to a given input and re-projects the angle error to adjust the sensor frame pose (position and orientation).
The fitting is not done "exact" and is not super accurate (~0.2? to ~1.0 degree accuracy, depending on the number of iterations) and seems to work only for "small" angle differences (< ~12 degree?).
Should work well and accurately enough for most real life __mobile__ robotic systems though.

The calculations are done in the 2D image space for fast computation (< ~10% CPU usage, input: 640x480@30Hz with Intel Haswell (2014) i7-4790 @ 3.6Ghz).

# How it works

Pre-calculate multipliers ``m_px`` and ``m_py`` for estimating the angles later:

1. Calculate planes which correspond to the maximum deviation ``e_max`` we expect for the ground plane
   1. Four planes: ± ``e_max`` around x and y axis
1. Project planes back into sensor image space (aka "depth image") (= plane images)
1. Calculate the *sum of differences* ``d_px``, ``d_py`` between ± x and ± y plane images
1. Calculate the multipliers with ``m_px = e_max / d_px``, ``m_py = e_max / d_py``

![multiplier calculation](https://raw.githubusercontent.com/AlexReimann/plane_calibration/devel/doc/plane_calibration_pre.png)

Estimate the angles:
1. Start with ``e = e_max``
2. Calculate the plane images as in the pre-calculation above for ``e``
3. Calculate the four *sums of differences* ``s_positive_px``, ``s_negative_px``, ``s_positive_py``, ``s_negative_py`` between input data and the ± ``e`` x/y image planes
4. Calculate the differences of the *sums of differences*:
   1. ``ds_px = s_negative_px - s_positive_px``
   2. ``ds_py = s_negative_py - s_positive_py``
5. Multiply ``ds_px`` and ``ds_py`` with the pre-computed multipliers ``m_px`` and ``m_py`` to get the estimated angle offsets ``offset_px`` and ``offset_py``
6. Use the ``offsets + some_buffer`` as estimated deviation ``e`` and repeat from _2._ ``n`` times
7. Sum up the offsets to get the plane xy orientation offsets
8. Calculate camera transformation to negate the calculated ground plane angle offsets

![difference calculation](https://raw.githubusercontent.com/AlexReimann/plane_calibration/devel/doc/plane_calibration.png)

Lazy / conservative update scheme:
1. Always check if input has"enough" data "at the right place" to fit a plane
   1. If not then do nothing and keep broadcasting old transformation
2. Calculate transformation once and keep using it if still fits the data
3. If transformation does not fit the data, calculate new
4. If new transformation fits the data, replace old transformation
5. Else if new transformation does not fit, keep old
6. Run the calibration only as fast as ``calibration_rate``, but publish the old transform every time we get input data

## Where do the multipliers come from / how does it work?
No really statisfying explanation available. Basically I ran some simulation tests and the angle error is __almost__ linear to ``ds_px`` (_difference of sum of differences_) mentioned above. ¯\\_(ツ)\_/¯
The residual error given a perfect ground plane could actually be easily estimated by a function fit, but the input data has some noise anyways and the iterative fitting works well enough. Maybe will do that in the future to converge with less iterations.

It should be mentioned that the multipliers depend on the distance of the fitted plane to the sensor. So it needs to be re-calculated every time this changes (in case someone wants to use this approach for something else).

# How to get started
The package is a [nodelet](http://wiki.ros.org/nodelet) and has a standalone launcher which just starts a nodelet manager and the nodelet for easy testing. It is set up to work out of a box if you run a sensor with the default [openni2 launcher](http://wiki.ros.org/openni2_launch).
But basically you want to attach the nodelet to the nodelet manager of your sensor driver for fast data transport.

The nodelet launcher has _args_ for the sensor topics and a path to a parameter file.
The package heavily uses [dynamic reconfigure ](http://wiki.ros.org/dynamic_reconfigure) for changing parameters on the fly. If the parameters are not specified in the parameter file, the defaults from the config file ``cfg/`` are used.

There is a lot of debug information (gets a bit spammy, but explains always why something fails) if the reconfigure _debug_ flag is set.
Instead of the _ground_frame_ (default: ``base_footprint``), also a manual ground transformation (from the reconfigure) can be used by checking the _use_manual_ground_transform_. The translation of the transformation has to be kind of accurate, same goes for the _Z_ orientation. If debug is enabled a TF transform will be showing the pose.

If debug is enabled a lot of topics and some transforms are getting published with information of the intermediate steps of the calibration (input filtering, initial starting plane, ground plane estimation).
Also variables which are checked for thresholding and limits are published as topics and can be introspected with e.g. [rqt_plot](http://wiki.ros.org/rqt_plot).

Play around with the _iterations_ parameter if the plane does not fit well enough. It can use a lot of CPU though and the default number of iterations (``3``) works reasonably well if the initial position is okish.
