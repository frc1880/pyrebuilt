from components.vision import Vision


class VisionCoordinator:
    red_vision: Vision
    white_vision: Vision
    shooter_vision: Vision

    def execute(self) -> None:
        if self.onField():
            # One camera is initialised, so turn on multi tag flag for all
            self.red_vision._has_seen_multitag = True
            self.white_vision._has_seen_multitag = True
            self.shooter_vision._has_seen_multitag = True

    def onField(self):
        return (
            self.red_vision._has_seen_multitag
            or self.white_vision._has_seen_multitag
            or self.shooter_vision._has_seen_multitag
        )
