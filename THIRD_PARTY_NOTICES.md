# Third-Party Notices

Most first-party ROS packages declare Apache-2.0 in their package.xml files.

The Go2W description assets in `src/go2w_description` originate from
[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros),
under the upstream BSD-3-Clause license. The license text is included in
`src/go2w_description/LICENSE`. This repository ports the package metadata to
ROS 2 ament and adds a separate simulation wrapper; it does not claim ownership
of Unitree meshes or represent an official Unitree simulator.

ROS 2, Nav2, SLAM Toolbox, Gazebo and Python dependencies retain their own licenses.
The independent AGV and generic quadruped models do not represent a certified
manufacturer-specific vehicle. No M20 Pro manufacturer model is included.
