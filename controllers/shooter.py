from magicbot import StateMachine, state

from components.ballistics import Ballistics
from components.drivetrain import Drivetrain
from components.indexer import Indexer


class ShooterController(StateMachine):
    ballistics: Ballistics
    drivetrain: Drivetrain
    indexer: Indexer

    @state(first=True)
    def aligning(self) -> None:
        # Point at the target
        self.drivetrain.track_heading(self.ballistics.solution().bearing)
        if self.drivetrain.is_aligned_with_hub():
            self.next_state("shooting")

    @state
    def shooting(self) -> None:
        # Check to see that we are still aligned with the goal
        # This is important if we are being defended
        self.drivetrain.track_heading(self.ballistics.solution().bearing)
        if not self.drivetrain.is_aligned_with_hub():
            self.next_state_now("aligning")
        else:
            # We are still aligned, so keep shooting
            self.indexer.feed()
