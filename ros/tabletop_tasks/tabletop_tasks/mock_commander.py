"""Mock commander."""

import numpy as np
import time


class PoissonProcess():
    def __init__(self, rate=0.1):
        self._rate = rate
        self._t_last_query = time.time()
        self._t_last_sample = time.time()
        
    def __call__(self):
        current_time = time.time()
        elapsed_time = current_time - self._t_last_query
        self._t_last_query = current_time
        # Exponential sample
        sample = np.random.exponential(1/self._rate)
        if sample < elapsed_time:
            self._t_last_sample = current_time
        return self._t_last_sample


class Commander:
    
    def __init__(self, hand_fixation_rate=0.1, flic_button_rate=0.1):
        self._task = None
        self._hand_fixation_process = PoissonProcess(hand_fixation_rate)
        self._flic_button_process = PoissonProcess(flic_button_rate)
    
    def smartglass_occlude(self):
        print("    Occluding smartglass")
    
    def smartglass_reveal(self):
        print("    Revealing smartglass")
    
    def arm_door_open(self):
        print("    Opening arm door")
        
    def arm_door_close(self):
        print("    Closing arm door")
    
    def reward(self, duration_ms):
        print(f"    Rewarding for {duration_ms} ms")
    
    def fetch_object(self, object_id, object_pose):
        # Note: We may want an intermediate level here, e.g. "ObjectMap",
        # to handle converting the fetch command to a series of waypoints, based
        # on the rig configuration. I don't know if this is best done as an
        # argument to ForagingTask or in the Commander node.
        print(f"    Fetching object {object_id} at pose {object_pose}")
    
    def return_object(self, object_id):
        print(f"    Returning object {object_id}")
        
    def move_to_position_sync(self, position):
        print(f"    Moving to position {position}")
    
    def t_hand_fixation_off(self):
        return self._hand_fixation_process()
    
    def t_flic_button(self):
        return self._flic_button_process()