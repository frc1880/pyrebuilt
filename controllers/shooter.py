from magicbot import StateMachine, state

from components.drivetrain import Drivetrain


class ShooterController(StateMachine):
    drivetrain: Drivetrain

    @state(first=True)
    def aligning(self) -> None:
        # Point at the target
        self.drivetrain.track_hub()
        if self.drivetrain.is_aligned_with_hub():
            self.next_state("shooting")

    @state
    def shooting(self) -> None:
        # Check to see that we are still aligned with the goal
        # This is important if we are being defended
        self.drivetrain.track_hub()
        if not self.drivetrain.is_aligned_with_hub():
            self.next_state_now("aligning")
        else:
            # We are still aligned, so keep shooting
            pass
