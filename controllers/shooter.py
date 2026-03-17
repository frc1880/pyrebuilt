from magicbot import StateMachine, state

from components.ballistics import Ballistics
from components.drivetrain import Drivetrain
from components.indexer import Indexer
from utilities import game, positions


class ShooterController(StateMachine):
    ballistics: Ballistics
    drivetrain: Drivetrain
    indexer: Indexer

    @state(first=True)
    def aligning(self) -> None:
        if not self.ballistics.is_within_range():
            return
        # Point at the target
        self.drivetrain.track_heading(
            positions.shooter_to_hub(self.drivetrain.pose()).radians()
        )
        if self.drivetrain.is_aligned():
            self.next_state("shooting")

    @state
    def shooting(self) -> None:
        # Check to see that we are still aligned with the goal
        # This is important if we are being defended
        self.drivetrain.track_heading(
            positions.shooter_to_hub(self.drivetrain.pose()).radians()
        )
        if (
            not self.drivetrain.is_aligned()
            or not self.ballistics.is_within_range()
            or not game.is_hub_active()
        ):
            self.next_state_now("aligning")
        else:
            # We are still aligned, so keep shooting
            self.indexer.feed()
