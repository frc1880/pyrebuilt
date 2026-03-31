from utilities.game import ShiftInfo, _shift_info_with_args


def test_hub_active() -> None:
    # In transition both are active
    assert _shift_info_with_args(135, True) == ShiftInfo(True, 5)
    assert _shift_info_with_args(135, False) == ShiftInfo(True, 5)

    # First shift
    assert _shift_info_with_args(115, True) == ShiftInfo(False, 10)
    assert _shift_info_with_args(115, False) == ShiftInfo(True, 10)

    # Second shift
    assert _shift_info_with_args(90, True) == ShiftInfo(True, 10)
    assert _shift_info_with_args(90, False) == ShiftInfo(False, 10)

    # Third shift
    assert _shift_info_with_args(65, True) == ShiftInfo(False, 10)
    assert _shift_info_with_args(65, False) == ShiftInfo(True, 10)

    # Fourth shift
    assert _shift_info_with_args(40, True) == ShiftInfo(True, 10)
    assert _shift_info_with_args(40, False) == ShiftInfo(False, 10)

    # End game both active
    assert _shift_info_with_args(20, True) == ShiftInfo(True, 20)
    assert _shift_info_with_args(20, False) == ShiftInfo(True, 20)
