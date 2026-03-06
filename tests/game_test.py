from utilities.game import _time_to_hub_active_with_args


def test_hub_active() -> None:
    # In transition both are active
    assert _time_to_hub_active_with_args(135, True) == 0
    assert _time_to_hub_active_with_args(135, False) == 0

    # First shift
    assert _time_to_hub_active_with_args(115, True) == 0
    assert _time_to_hub_active_with_args(115, False) == 10

    # Second shift
    assert _time_to_hub_active_with_args(90, True) == 10
    assert _time_to_hub_active_with_args(90, False) == 0

    # Third shift
    assert _time_to_hub_active_with_args(65, True) == 0
    assert _time_to_hub_active_with_args(65, False) == 10

    # Fourth shift
    assert _time_to_hub_active_with_args(40, True) == 10
    assert _time_to_hub_active_with_args(40, False) == 0

    # End game both active
    assert _time_to_hub_active_with_args(20, True) == 0
    assert _time_to_hub_active_with_args(20, False) == 0
