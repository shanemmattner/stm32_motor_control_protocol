"""Example test cases."""

import pytest


class TestExample:
    """Example test class."""

    def test_imports(self):
        """Test that main modules can be imported."""
        from st_mcp import MinimalMotorControl
        from st_mcp.commands import (
            StartMotorCommand,
            StopMotorCommand,
            Monitor1Data,
        )

        assert MinimalMotorControl is not None
        assert StartMotorCommand is not None
        assert StopMotorCommand is not None
        assert Monitor1Data is not None

    def test_version(self):
        """Test version is accessible."""
        from st_mcp import __version__

        assert __version__ == "0.1.0"
