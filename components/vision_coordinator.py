from components.vision import Vision


class VisionCoordinator:
    red_vision: Vision
    white_vision: Vision
    shooter_vision: Vision

    def execute(self) -> None:
        # Replaced camera check with a function
        if self.on_field():
            # One camera is initialised, so turn on multi tag flag for all
            self.red_vision._has_seen_multitag = True
            self.white_vision._has_seen_multitag = True
            self.shooter_vision._has_seen_multitag = True

    # Checks if one camera has seen a apriltag,, which would indicate if on field
    def on_field(self):
        return (
            self.red_vision._has_seen_multitag
            or self.white_vision._has_seen_multitag
            or self.shooter_vision._has_seen_multitag
        )
