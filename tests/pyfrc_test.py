"""
This test module imports tests that come with pyfrc, and can be used
to test basic functionality of just about any robot.
"""

# NB: autonomous is tested separately in autonomous_test.py
from pyfrc.tests import (
    test_disabled,
    test_operator_control,
)

# Make pyflakes happy about our imports.
__all__ = (
    "test_disabled",
    "test_operator_control",
)
