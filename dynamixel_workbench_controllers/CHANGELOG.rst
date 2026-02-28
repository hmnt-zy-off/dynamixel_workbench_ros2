^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_workbench_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-02-27)
------------------
* Created dynamixel_workbench_controllers package.
* Ported find_dynamixel utility from ROS1 to ROS2.
* Ported main Dynamixel Workbench Controller node to ROS 2.
* Implemented rclcpp::Node for proper node initialization.
* Updated logging to RCLCPP.
* Implemented thread-safe hardware access using std::mutex to prevent port conflict for the controller.
* Verified execution & build.

0.3.0 (2026-02-28)
------------------
* Added dynamixel_workbench_operators package.
* Version bump to align with full ROS 2 stack release.