"""Motor control commands for MCP protocol."""

# Registry commands (new)
from st_mcp.commands.registry import (
    ReadRegisterCommand,
    WriteRegisterCommand,
)

# Old motor and monitor commands (not yet refactored)
# from st_mcp.commands.motor import (...)
# from st_mcp.commands.monitor import (...)

__all__ = [
    "ReadRegisterCommand",
    "WriteRegisterCommand",
]
