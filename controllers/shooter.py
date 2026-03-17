import math

from magicbot import StateMachine, state

from components.ballistics import Ballistics
from components.drivetrain import Drivetrain
from components.indexer import Indexer
from utilities import game, positions


class ShooterController(StateMachine):
    ballistics: Ballistics
    drivetrain: Drivetrain
    indexer: Indexer

    def _heading(self) -> float:
        # If we are in our alliance zone, aim at the hub
        # Otherwise fire back at the alliance wall as a passing shot
        if game.is_in_alliance_zone(self.drivetrain.pose()):
            return positions.shooter_to_hub(self.drivetrain.pose()).radians()
        else:
            return math.radians(-90.0) if game.is_red() else math.radians(90.0)

    def _can_shoot(self) -> bool:
        # We can shoot if we are in our zone and the hub is active,
        # or we are outside our zone at any time
        in_zone = positions.is_in_alliance_zone(self.drivetrain.pose())
        return (game.is_hub_active() and in_zone) or not in_zone

    @state(first=True)
    def aligning(self) -> None:
        if not self._can_shoot():
            return
        # Point at the target
        self.drivetrain.track_heading(self._heading())
        if self.drivetrain.is_aligned():
            self.next_state("shooting")

    @state
    def shooting(self) -> None:
        # Check to see that we are still aligned with the goal
        # This is important if we are being defended
        self.drivetrain.track_heading(self._heading())
        if not (self._can_shoot() and self.drivetrain.is_aligned()):
            self.next_state_now("aligning")
        else:
            # We are still aligned, so keep shooting
            self.indexer.feed()
