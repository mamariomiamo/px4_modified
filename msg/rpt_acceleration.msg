# RPT control acceleration setpoint and current acceleration in NED frame
# this is used for logging purpose

# Acceleration in NED frame
float32 ax        # North velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 ay        # East velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 az        # Down velocity derivative in NED earth-fixed frame, (metres/sec^2)

# Acceleration setpoint in NED frame
float32 ax_sp        # North velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 ay_sp        # East velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 az_sp       # Down velocity derivative in NED earth-fixed frame, (metres/sec^2)