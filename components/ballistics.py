from components.drivetrain import Drivetrain


class Ballistics:
    """
    Keep track of our current position and use it to calculate
    the correct flywheel speed.
    """

    # We will need the drivebase so that we can get our current position
    drivetrain: Drivetrain

    def __init__(self) -> None:
        self._shooter_speed = 0.0

    def shooter_speed(self) -> float:
        return self._shooter_speed

    def execute(self) -> None:
        """
        Calculate the required speed and store it in the _shooter_speed variable.
        """
        pass
