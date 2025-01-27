"""Foraging task."""

import abc
import enum
import time


class ForagingState(enum.Enum):
    """Foraging state."""

    IDLE = 0
    FETCH = 1
    STIMULUS = 2
    DELAY = 3
    RESPONSE = 4
    REVEAL = 5
    RETURN = 6


class ForagingTask(abc.ABC):
    def __init__(
        self,
        commander,
        trial_generator,
        fixation_duration=0.5,
        stimulus_duration=0.5,
        delay_duration=0.5,
        response_timeout=10.0,
        reward_duration_ms=100,
        reveal_duration=0.5,
    ):
        self._commander = commander
        self._trial_generator = trial_generator
        self._fixation_duration = fixation_duration
        self._stimulus_duration = stimulus_duration
        self._delay_duration = delay_duration
        self._response_timeout = response_timeout
        self._reward_duration_ms = reward_duration_ms
        self._reveal_duration = reveal_duration
        self._state = ForagingState.IDLE

    def _fetch(self):
        """Fetch object for trial."""
        # Sample new trial
        self._trial_spec = self._trial_generator()
        self._trial_feedback = dict(
            broke_fixation=False,
            reaction_time=None,
            timeout=None,
        )
        print("New trial")

        # Make smartglass opaque
        self._commander.smartglass_occlude()

        # Fetch object
        object_id = self._trial_spec.object_id
        object_pose = self._trial_spec.object_pose
        self._commander.fetch_object(object_id, object_pose)

        # Wait for hand fixation
        while True:
            t_hand_off = self._commander.t_hand_fixation_off()
            if time.time() - t_hand_off > self._fixation_duration:
                break
            time.sleep(0.01)

        # Transition to stimulus state
        self._state = ForagingState.STIMULUS

    def _stimulus(self):
        """Present stimulus."""
        # Reveal stimulus
        self._commander.smartglass_reveal()

        # Wait for stimulus duration, terminating early if hand fixation is
        # broken
        t_stimulus_start = time.time()
        fixation_start_time = t_stimulus_start - self._fixation_duration
        while time.time() - t_stimulus_start < self._stimulus_duration:
            t_hand_off = self._commander.t_hand_fixation_off()
            if t_hand_off > fixation_start_time:
                self._trial_feedback["broke_fixation"] = True
                break
            time.sleep(0.01)

        # Transition to delay state or terminate trial
        if self._trial_feedback["broke_fixation"]:
            self._state = ForagingState.RETURN
        else:
            self._state = ForagingState.DELAY

    def _delay(self):
        """Delay period."""
        # Occlude smartglass if necessary
        if self._trial_spec.occlude:
            self._commander.smartglass_occlude()

        # Wait for delay duration
        time.sleep(self._delay_duration)

        # Wait for stimulus duration, terminating early if hand fixation is
        # broken
        t_delay_start = time.time()
        fixation_start_time = (
            t_delay_start - self._fixation_duration - self._stimulus_duration
        )
        while time.time() - t_delay_start < self._delay_duration:
            t_hand_off = self._commander.t_hand_fixation_off()
            if t_hand_off > fixation_start_time:
                self._trial_feedback["broke_fixation"] = True
                break
            time.sleep(0.01)

        # Transition to response state or terminate trial
        if self._trial_feedback["broke_fixation"]:
            self._state = ForagingState.RETURN
        else:
            self._state = ForagingState.RESPONSE

    def _response(self):
        """Response period."""
        # Open arm door
        self._commander.arm_door_open()

        # Wait for response
        response_start_time = time.time()
        timeout = True
        while time.time() < response_start_time + self._response_timeout:
            t_response = self._commander.t_flic_button()
            if t_response > response_start_time:
                timeout = False
                self._trial_feedback["reaction_time"] = (
                    t_response - response_start_time
                )
                self._commander.reward(self._reward_duration_ms)
                break
            time.sleep(0.01)
        self._trial_feedback["timeout"] = timeout

        # Transition to reveal state or terminate trial
        if timeout:
            self._state = ForagingState.RETURN
        else:
            self._state = ForagingState.REVEAL

    def _reveal(self):
        """Reveal object."""
        # Reveal object
        self._commander.smartglass_reveal()

        # Wait for reveal duration
        time.sleep(self._reveal_duration)

        # Transition to return state
        self._state = ForagingState.RETURN

    def _return(self):
        """Return object."""
        # Give feedback to trial generator
        self._trial_generator.feedback(
            self._trial_spec, **self._trial_feedback
        )

        # Close arm door
        self._commander.arm_door_close()

        # Occlude smartglass
        self._commander.smartglass_occlude()

        # Return object
        self._commander.return_object(self._trial_spec.object_id)

        # Transition to fetch state
        self._state = ForagingState.FETCH

    def _return_async(self):
        arm_door_task = self._commander.arm_door_close(blocking=False)
        smartglass_occlude_task = self._commander.smartglass_occlude(
            blocking=False
        )
        return_object_task = self._commander.return_object(
            self._trial_spec.object_id, blocking=False
        )

        self._trial_generator.feedback(
            self._trial_spec, **self._trial_feedback
        )

        arm_door_task()
        smartglass_occlude_task()
        return_object_task()

        self._state = ForagingState.FETCH

    def _run_loop(self):
        """Run a trial."""
        if self._state == ForagingState.FETCH:
            self._fetch()
        elif self._state == ForagingState.STIMULUS:
            self._stimulus()
        elif self._state == ForagingState.DELAY:
            self._delay()
        elif self._state == ForagingState.RESPONSE:
            self._response()
        elif self._state == ForagingState.REVEAL:
            self._reveal()
        elif self._state == ForagingState.RETURN:
            self._return()

    def run_async_example(self):
        self._trial_spec = self._trial_generator()
        self._trial_feedback = dict(
            broke_fixation=False,
            reaction_time=None,
            timeout=None,
        )
        print("New trial")

        # Make smartglass opaque
        self._commander.smartglass_occlude()

        # Fetch object
        object_id = self._trial_spec.object_id
        object_pose = self._trial_spec.object_pose
        fetch_task = self._commander.fetch_object_async(object_id, object_pose)

    def start(self):
        self._state = ForagingState.FETCH
        while True:
            self._run_loop()
            time.sleep(0.01)
