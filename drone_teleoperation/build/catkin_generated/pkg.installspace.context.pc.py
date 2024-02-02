# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rospy;std_msgs;mav_manager;trackers_msgs;quadrotor_msgs;scene_understanding_pkg_msgs;scene_understanding_pkg".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmav_manager".split(';') if "-lmav_manager" != "" else []
PROJECT_NAME = "drone_teleoperation"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
