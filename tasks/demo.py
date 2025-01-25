"""Demo foraging task with mock commander."""

import foraging_task
import mock_commander
from trial_generators import blocked_cup_drawer


def main():
    """Main function."""
    # Setup trial generator
    trial_generator = blocked_cup_drawer.BlockedCupDrawer()
    
    # Setup commander
    commander = mock_commander.Commander()
    
    # Setup foraging task
    task = foraging_task.ForagingTask(
        commander=commander,
        trial_generator=trial_generator,
    )
    
    # Run task
    task.start()

    
if __name__ == "__main__":
    main()