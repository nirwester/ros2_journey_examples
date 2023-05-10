Compile with "colcon build" note: requires ROS 2 >= Iron
source install/setup.sh

Terminal 1: ros2 run lazy_subscriber publisher

To set the log level to DEBUG

Terminal 2: ros2 service call /minimal_publisher/set_logger_levels rcl_interfaces/srv/SetLoggerLevels "{levels:[name: '', level: 10]}"

To set different log levels, change the used constant according to the following definitions:

LoggerLevel[] levels
	uint8 LOG_LEVEL_UNKNOWN = 0
	uint8 LOG_LEVEL_DEBUG = 10
	uint8 LOG_LEVEL_INFO = 20
	uint8 LOG_LEVEL_WARN = 30
	uint8 LOG_LEVEL_ERROR = 40
	uint8 LOG_LEVEL_FATAL = 50
