# Single binocular gaze sample from the SR Research Eyelink eye tracker.
# Published by: eyelink node (nodes/eyelink.py, topic ~/sample) at up to 1000 Hz.
# Consumed by: EyelinkInterface (interfaces/eyelink.py) via action feedback;
#              also batched into EyelinkArray and written to rosbag.

std_msgs/Header header      # ROS timestamp assigned when the sample is dequeued from pylink
uint32 eyelink_time_ms      # Eyelink-internal timestamp in milliseconds (from tracker clock via pylink Sample.getTime())
float32 left_x              # Left eye raw pupil X position in Eyelink camera pixel units; MISSING_DATA (-32768) when eye not tracked
float32 left_y              # Left eye raw pupil Y position in Eyelink camera pixel units; MISSING_DATA (-32768) when eye not tracked
float32 left_pupil          # Left eye pupil diameter in arbitrary Eyelink units (diameter mode); MISSING_DATA (-32768) when unavailable
float32 right_x             # Right eye raw pupil X position in Eyelink camera pixel units; MISSING_DATA (-32768) when eye not tracked
float32 right_y             # Right eye raw pupil Y position in Eyelink camera pixel units; MISSING_DATA (-32768) when eye not tracked
float32 right_pupil         # Right eye pupil diameter in arbitrary Eyelink units (diameter mode); MISSING_DATA (-32768) when unavailable
uint16 input                # Eyelink digital input port value (from Sample.getInput()); used to detect external TTL triggers (e.g., 255=idle, 247=trigger)
