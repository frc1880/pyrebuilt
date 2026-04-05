import math

import wpilib
from magicbot import StateMachine, state

from components.ballistics import Ballistics
from components.drivetrain import Drivetrain
from components.indexer import Indexer
from components.intake import Intake
from components.shooter import Shooter
from utilities import game, positions


class ShooterController(StateMachine):
    ballistics: Ballistics
    drivetrain: Drivetrain
    indexer: Indexer
    intake: Intake
    shooter: Shooter

    def _heading(self) -> float:
        # If we are in our alliance zone, aim at the hub
        # Otherwise fire back at the alliance wall as a passing shot
        if positions.is_in_alliance_zone(self.drivetrain.pose()):
            return self.ballistics.solution().bearing
        else:
            return math.radians(180.0) if game.is_red() else math.radians(0.0)

    def _can_shoot(self) -> bool:
        # We can shoot if we are in our zone and the hub is active,
        # or we are outside our zone at any time
        if wpilib.DriverStation.isAutonomousEnabled():
            # Always shoot in auto
            return True
        in_zone = positions.is_in_alliance_zone(self.drivetrain.pose())
        return (game.is_hub_active() and in_zone) or not in_zone

    @state(first=True)
    def aligning(self) -> None:
        if not self._can_shoot():
            return
        # Point at the target
        self.drivetrain.track_heading(self._heading())
        self.shooter.shoot()  # Spin up flywheels if not in alliance zone
        if self.drivetrain.is_aligned():
            self.next_state("shooting")

    @state
    def shooting(self) -> None:
        # Check to see that we are still aligned with the goal
        # This is important if we are being defended
        self.drivetrain.track_heading(self._heading())
        self.shooter.shoot()  # Spin up flywheels if not in alliance zone
        if not self.drivetrain.is_aligned():
            self.next_state_now("aligning")
        else:
            # We are still aligned, so keep shooting
            if self.shooter.at_speed():
                self.indexer.feed()
                self.intake.spin()
