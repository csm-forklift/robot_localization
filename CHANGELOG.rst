^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#13 <https://github.com/clearpathrobotics/robot_localization/issues/13>`_ from clearpathrobotics/CORE-2308-format
  Added format to ROS_LOG so the code compiles with -Werror=format-security
* Removed unneeded IDE temporary file
  Added format to ROS_LOG so the code compiles with -Werror=format-security
* Contributors: Jason Mercer

2.2.2 (2015-10-30)
------------------
* Merge pull request `#12 <https://github.com/clearpathrobotics/robot_localization/issues/12>`_ from clearpathrobotics/CORE-2288-catch-lock-error
  Waiting for worker thread to join main thread before destroying the object.
* Waiting for worker thread to join main thread before destroying the object.
* Merge pull request `#11 <https://github.com/clearpathrobotics/robot_localization/issues/11>`_ from clearpathrobotics/check_can_transform_odom_base_link
  Check that odom->base_link transform is available
* Check that odom->base_link transform is available
  This silence the ERROR log msg that happens if the odom->base_link
  transform isn't available later in the code, in cases where the odom
  broadcaster is still starting/warming up.
* Merge pull request `#10 <https://github.com/clearpathrobotics/robot_localization/issues/10>`_ from clearpathrobotics/add_high_covariance_volume_check
  Add high covariance volume delta check
* Add test script for input covariance
* Add covariance (delta) volume check
  This also silence the check for the large covariance check on single
  covariance elements, which reports harmless warning in some case, e.g.
  pure wheel (pose) odometry which grows without bounds but it's a normal
  and intended behaviour
* Merge pull request `#9 <https://github.com/clearpathrobotics/robot_localization/issues/9>`_ from clearpathrobotics/CHECC-196-tf-param
  Adding parameter to make transform publishing optional
* Fixing catch statement for failed tf lookup
* Adding printout of new parameter
* Adding parameter to make transform publishing optional
* Merge pull request `#8 <https://github.com/clearpathrobotics/robot_localization/issues/8>`_ from clearpathrobotics/remove_qtcreator_file
  Delete CMakeLists.txt.user
* Delete CMakeLists.txt.user
* Merge pull request `#7 <https://github.com/clearpathrobotics/robot_localization/issues/7>`_ from clearpathrobotics/CORE-2178-diagnostic-mutex
  Added mutex guards …
* Enhanced documentation.
* Aesthetics
* Added mutex guards around access to staticDiagnostics_ and dynamicDiagnostics_ in RobotLocalization::RosFilter. Memory corruption without this is possible and was observed on the VM.
* Merge pull request `#6 <https://github.com/clearpathrobotics/robot_localization/issues/6>`_ from clearpathrobotics/fix_locking_bug
  Fix a bug with mutex locking
* Fix a bug with mutex locking
* Merge pull request `#5 <https://github.com/clearpathrobotics/robot_localization/issues/5>`_ from clearpathrobotics/core_1695_delay_new
  Core 1695 delay new
* Style modifications
* Changed the way the results are published so that the integration
  results of two messages that have the same time stamp don't get
  published separately.
* Merge the changes from core_1695_delay into indigo-devel
* Merging warning removal changes from master
* Merge pull request `#236 <https://github.com/clearpathrobotics/robot_localization/issues/236>`_ from ayrton04/master
  Merging master branch from @ayrton04
* Merging EKF Jacobian fix from upstream
* Merging PR `#233 <https://github.com/clearpathrobotics/robot_localization/issues/233>`_ (Jacobian fix) from master
* Merge pull request `#233 <https://github.com/clearpathrobotics/robot_localization/issues/233>`_ from adnanademovic/master
  Fix Jacobian for EKF.
* Fix Jacobian for EKF.
* Merge branch 'indigo-devel'
* Removing warning about orientation variables when only their velocities are measured
* Merge pull request `#230 <https://github.com/clearpathrobotics/robot_localization/issues/230>`_ from ayrton04/indigo-devel
  Fixing issue `#220 <https://github.com/clearpathrobotics/robot_localization/issues/220>`_
* Merge pull request `#229 <https://github.com/clearpathrobotics/robot_localization/issues/229>`_ from ayrton04/master
  Fixing issue `#220 <https://github.com/clearpathrobotics/robot_localization/issues/220>`_
* Merge branch 'indigo-devel'
* Checking for -1 in IMU covariances and ignoring relevant message data. Also adding unit tests for this case. This checkin resolves issue `#220 <https://github.com/clearpathrobotics/robot_localization/issues/220>`_.
* Merge pull request `#227 <https://github.com/clearpathrobotics/robot_localization/issues/227>`_ from ayrton04/indigo-devel
  Post-linting merge
* Merge pull request `#226 <https://github.com/clearpathrobotics/robot_localization/issues/226>`_ from ayrton04/master
  Post-linting merge
* Merge branch 'jade-devel'
* Merge branch 'jade-devel' into indigo-devel
* Typo fix in CMakeLists.txt
* Merge remote-tracking branch 'upstream/jade-devel' into jade-devel
* Merge remote-tracking branch 'upstream/master'
* Merge remote-tracking branch 'upstream/indigo-devel' into indigo-devel
* Correcting CMakeLists.txt and package.xml to comply with catkin_lint
* Merge branch 'master' into indigo-devel
* Final roslint checkin.
* Enough linting to make a sweater.
* In the midst of roslinting everything
* Updating package.xml to format two and cleaning up CMakeLists
* Merge pull request `#217 <https://github.com/clearpathrobotics/robot_localization/issues/217>`_ from ayrton04/jade-devel
  Merging ayrton04's branches
* Merge pull request `#216 <https://github.com/clearpathrobotics/robot_localization/issues/216>`_ from ayrton04/indigo-devel
  Merging ayrton04's branches
* Merge pull request `#215 <https://github.com/clearpathrobotics/robot_localization/issues/215>`_ from ayrton04/master
  Merging ayrton04's branches
* Merge branch 'jade-devel'
* Merge branch 'jade-devel' into indigo-devel
* Adding base_link to datum specification, and fixing bug with order of measurement handling when a datum is specified. Also added check to make sure IMU data is transformable before using it.
* 2.2.1
* Updating changelong prior to 2.2.1
* Merge pull request `#212 <https://github.com/clearpathrobotics/robot_localization/issues/212>`_ from ayrton04/jade-devel
  Merging preparePose changes from @ayrton04's branch
* Merge pull request `#210 <https://github.com/clearpathrobotics/robot_localization/issues/210>`_ from ayrton04/master
  Merging preparePose changes from @ayrton04's branch
* Merge branch 'indigo-devel'
* Merge branch 'indigo-devel' into jade-devel
* Updating version before release
* Merge pull request `#206 <https://github.com/clearpathrobotics/robot_localization/issues/206>`_ from ayrton04/master
  Merging ayrton04's branches
* Merge pull request `#205 <https://github.com/clearpathrobotics/robot_localization/issues/205>`_ from ayrton04/jade-devel
  Merging ayrton04's branches
* Merge branch 'indigo-devel'
* Merge branch 'indigo-devel' into jade-devel
* Merge branch 'indigo-devel' into jade-devel
* Merge branch 'indigo-devel'
* Merge branch 'indigo-devel'
* Merge branch 'indigo-devel' into jade-devel
* Updating changelog prior to release of version 2.2.0
* Updating changelog prior to jade release of version 2.2.0
* Merge branch 'indigo-devel'
* Merge branch 'indigo-devel' into jade-devel
* Merge remote-tracking branch 'downstream/master'
* Merge remote-tracking branch 'downstream/jade-devel' into jade-devel
* Merge branch 'indigo-devel'
* Merge branch 'indigo-devel' into jade-devel
* Merge branch 'indigo-devel' into jade-devel
* Merge remote-tracking branch 'upstream/master'
* Merge remote-tracking branch 'upstream/jade-devel' into jade-devel
* Merge pull request `#199 <https://github.com/clearpathrobotics/robot_localization/issues/199>`_ from cra-ros-pkg/master
  Merging change from master into relevant branches
* Merge pull request `#195 <https://github.com/clearpathrobotics/robot_localization/issues/195>`_ from ayrton04/jade-devel
  Merging datum addition for navsat_transform_node
* Merge branch 'indigo-devel' into jade-devel
* Merge branch 'indigo-devel' into jade-devel
* Merge pull request `#192 <https://github.com/clearpathrobotics/robot_localization/issues/192>`_ from ayrton04/jade-devel
  Adding rosbag dependency
* Merge branch 'indigo-devel' into jade-devel
* Merge pull request `#188 <https://github.com/clearpathrobotics/robot_localization/issues/188>`_ from ayrton04/jade-devel
  Jade devel
* Merge branch 'indigo-devel' into jade-devel
* Merge branch 'jade-devel' of https://github.com/ayrton04/robot_localization into jade-devel
* Merge pull request `#184 <https://github.com/clearpathrobotics/robot_localization/issues/184>`_ from ayrton04/jade-devel
  Merging tf2_geometry_msgs dependency change
* Merge branch 'indigo-devel' into jade-devel
* Merge pull request `#179 <https://github.com/clearpathrobotics/robot_localization/issues/179>`_ from ayrton04/jade-devel
  tf2 migration, test fixes, removal of gps_common dependency, enforcing ENU standard for navsat_transform_node
* Merge branch 'indigo-devel' into jade-devel
* Merge branch 'indigo-devel' into jade-devel
* Merging diagnostic_updater dependency changes
* Merge branch 'indigo-devel' into jade-devel
* Merge branch 'indigo-devel' into jade-devel
* Merging diagnostics from indigo-devel
* Merge branch 'indigo-devel' into jade-devel
* Merge remote-tracking branch 'upstream/jade-devel' into jade-devel
* Merge branch 'master' into jade-devel
* Merge remote-tracking branch 'downstream/master' into jade-devel
* Merge branch 'indigo-devel' into jade-devel
* Merge branch 'indigo-devel' into jade-devel
* Merge remote-tracking branch 'downstream/jade-devel' into jade-devel
* Merge remote-tracking branch 'downstream/jade-devel' into jade-devel
* Merge remote-tracking branch 'downstream/accuracy_fixes' into jade-devel
* Contributors: Adel Fakih, Adnan Ademovic, Enrique Fernandez, Enrique Fernández Perdomo, James Servos, Jason Mercer, Tom Moore, afakihcpr, ayrton04

2.2.1 (2015-05-27)
------------------
* Fixed handling of IMU data w.r.t. differential mode and relative mode

2.2.0 (2015-05-22)
------------------
* Added tf2-friendly tf_prefix appending
* Corrected for IMU orientation in navsat_transform
* Fixed issue with out-of-order measurements and pose resets
* Nodes now assume ENU standard for yaw data
* Removed gps_common dependency
* Adding option to navsat_transform_node that enables the use of the heading from the odometry message instead of an IMU.
* Changed frame_id used in setPoseCallback to be the world_frame
* Optimized Eigen arithmetic for signficiant performance boost
* Migrated to tf2
* Code refactoring and reorganization
* Removed roll and pitch from navsat_transform calculations
* Fixed transform for IMU data to better support mounting IMUs in non-standard orientations
* Added feature to navsat_transform_node whereby filtered odometry data can be coverted back into navsat data
* Added a parameter to allow future dating the world_frame->base_link_frame transform.
* Removed deprecated differential setting handler
* Added relative mode
* Updated and improved tests
* Fixing source frame_id in pose data handling
* Added initial covariance parameter
* Fixed bug in covariance copyinh
* Added parameters for topic queue sizes
* Improved motion model's handling of angular velocities when robot has non-zero roll and pitch
* Changed the way differential measurements are handled
* Added diagnostics

2.1.7 (2015-01-05)
------------------
* Added some checks to eliminate unnecessary callbacks
* Updated launch file templates
* Added measurement outlier rejection
* Added failure callbacks for tf message filters
* Added optional broadcast of world_frame->utm transform for navsat_transform_node
* Bug fixes for differential mode and handling of Z acceleration in 2D mode

2.1.6 (2014-11-06)
------------------
* Added unscented Kalman filter (UKF) localization node
* Fixed map->odom tf calculation
* Acceleration data from IMUs is now used in computing the state estimate
* Added 2D mode

2.1.5 (2014-10-07)
------------------
* Changed initial estimate error covariance to be much smaller
* Fixed some debug output
* Added test suite
* Better compliance with REP-105
* Fixed differential measurement handling
* Implemented message filters
* Added navsat_transform_node

2.1.4 (2014-08-22)
------------------
* Adding utm_transform_node to install targets

2.1.3 (2014-06-22)
------------------
* Some changes to ease GPS integration
* Addition of differential integration of pose data
* Some documentation cleanup
* Added UTM transform node and launch file
* Bug fixes

2.1.2 (2014-04-11)
------------------
* Updated covariance correction formulation to "Joseph form" to improve filter stability.
* Implemented new versioning scheme.

2.1.1 (2014-04-11)
------------------
* Added cmake_modules dependency for Eigen support, and added include to silence boost::signals warning from tf include

