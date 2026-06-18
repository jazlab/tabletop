# Periodic sensor snapshot from the Teensy microcontroller (topic: teensy/sensor).
# Published by: teensy node (tabletop_micro firmware) or mock_teensy node at 100 Hz (SENSOR_PERIOD_MS=10).
# Consumed by: TeensyInterface (tabletop_rig/interfaces/teensy.py) and Commander
#              (_teensy_sensor_callback) for safety interlock decisions.

std_msgs/Header header                        # Timestamp set on the Teensy using synchronized ROS time (rmw_uros_epoch_nanos)
bool is_safety_laser_broken                   # True when the safety laser beam is interrupted (HIGH on SAFETY_LASER_STATE_PIN); blocks robot execution
bool is_left_arm_locked                       # True when the subject's left arm is physically seated in the restraint (debounced LOW on LEFT_ARM_LOCK_STATE_PIN)
bool is_right_arm_locked                      # True when the subject's right arm is physically seated in the restraint (debounced LOW on RIGHT_ARM_LOCK_STATE_PIN)
bool is_reward_active                         # True while the juice/reward solenoid is open (REWARD_CONTROL_PIN HIGH)
bool is_button_pressed                        # True while the subject's response button is pressed (LOW on BUTTON_STATE_PIN)
bool is_smartglass_revealed                   # True when the smartglass pane is transparent (SMARTGLASS_CONTROL_PIN HIGH)
uint16[5] left_tactile_glove_states           # 10-bit ADC readings (0-1023) for the 5 left-hand glove pressure sensors (pins A0-A4)
uint16[5] right_tactile_glove_states          # 10-bit ADC readings (0-1023) for the 5 right-hand glove pressure sensors (pins A5-A9)
bool sync_pulse_state                         # True while the sync pulse output (SYNC_PULSE_CONTROL_PIN) is HIGH; pulse duration ~100 ms, period ~1 s + jitter
builtin_interfaces/Time sync_pulse_last_time_on   # ROS timestamp of the most recent sync pulse rising edge
builtin_interfaces/Time sync_pulse_last_time_off  # ROS timestamp of the most recent sync pulse falling edge
builtin_interfaces/Time safety_laser_last_time_broken  # ROS timestamp of the last safety laser break event (interrupt-latched)
builtin_interfaces/Time button_last_time_pressed   # ROS timestamp of the last button press event (interrupt-latched)
