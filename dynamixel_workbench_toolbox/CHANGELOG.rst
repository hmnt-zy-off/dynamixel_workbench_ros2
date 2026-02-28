^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_workbench_toolbox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-02-20)
------------------
* Created dynamixel_workbench_toolbox package.
* Migrated dynamixel_workbench_toolbox from ROS 1 branch of dynamixel_workbench_toolbox.
* updated package.xml and CMakeLists.txt
* Changed uint8_t to vector array for removing Wvla Warnings in _driver.cpp
* Fixed the typos in the dynamixel_driver.cpp file log statement.
* Build

0.2.0 (2026-02-27)
------------------
* Fixed memory access bug in readRegister and syncWrite.
* Replaced incorrect vector object addresses with .data() pointers for DXL SDK compatibility.

0.3.0 (2026-02-28)
------------------
* Added dynamixel_workbench_operators package.
* Version bump to align with full ROS 2 stack release


