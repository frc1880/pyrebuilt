from wpimath.geometry import Transform2d


class Leds:
    """
    Light patterns for driver feedback.
    """

    def __init__(self) -> None:
        pass

    def missing_vision(self) -> None:
        pass

    def missing_auto(self) -> None:
        pass

    def wrong_start(self, error: Transform2d):
        # Light up various parts of the robot to show the direction to move it
        if error.rotation().radians() > 0:
            # Negative z rotation required
            pass
        elif error.rotation().radians() < 0:
            # Positive z rotation required
            pass
        else:
            # Rotation is okay, so give feedback on position
            # NB these need to work together, eg move forward and move left simultaneously
            if error.translation().x > 0:
                # Move back required
                pass
            elif error.translation().x < 0:
                # Move forward required
                pass
            if error.translation().y > 0:
                # Move right required
                pass
            elif error.translation().y < 0:
                # Move left required
                pass

    def disabled(self) -> None:
        pass

    def execute(self) -> None:
        pass
