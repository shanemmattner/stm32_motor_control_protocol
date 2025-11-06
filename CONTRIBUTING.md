# Contributing to STM32 Motor Control Protocol

Thank you for your interest in contributing! Here's how to get started.

## Development Setup

### Prerequisites
- Python 3.8+
- [uv](https://github.com/astral-sh/uv) (fast Python package manager)

### Quick Start

```bash
# Clone the repository
git clone git@github.com:shanemmattner/stm32_motor_control_protocol.git
cd stm32_motor_control_protocol

# Install development dependencies
make install-dev

# Run all checks
make all
```

## Development Workflow

### Code Style

We use modern Python tooling for consistency:

- **Format code**: `make format` (Black + isort)
- **Lint**: `make lint` (Ruff)
- **Type check**: `make type-check` (mypy)
- **Test**: `make test` (pytest)

Run all checks before committing:
```bash
make all
```

### Writing Tests

Place tests in the `tests/` directory with `test_*.py` naming:

```python
"""Test motor control commands."""

import pytest
from st_mcp.commands import StartMotorCommand


class TestMotorCommands:
    """Test motor control command classes."""

    def test_start_motor_command_creation(self):
        """Test StartMotorCommand initialization."""
        cmd = StartMotorCommand(motor_id=0)
        assert cmd.motor_id == 0
        assert cmd.command_id == StartMotorCommand.START
```

Run tests with coverage:
```bash
make test
```

### Adding Dependencies

Use `uv` to add dependencies:

```bash
# Add production dependency
uv add pyserial

# Add development dependency
uv add --dev pytest
```

## Pull Request Process

1. Create a feature branch: `git checkout -b feature/your-feature`
2. Make your changes with tests
3. Run `make all` to ensure code quality
4. Commit with clear messages (see below)
5. Push and create a pull request

## Commit Messages

Follow conventional commits:

```
type(scope): description

Optional detailed explanation

Fixes: #123
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation
- `test`: Tests
- `refactor`: Code refactoring
- `perf`: Performance improvement

Examples:
- `feat(motor): add velocity ramp control`
- `fix(protocol): correct ASPEP packet CRC calculation`
- `docs(readme): add installation instructions`

## Code Quality Standards

- All code must pass `ruff`, `black`, and `mypy` checks
- Tests required for new features
- Docstrings for public functions/classes
- Type hints for function parameters and returns
- Minimum 80% test coverage

## Documentation

- Update README.md for user-facing changes
- Add docstrings to all public APIs
- Update PROTOCOL_DOCUMENTATION.md for protocol changes
- Include code examples in docstrings

## Reporting Issues

Use GitHub Issues with:
- Clear description of the problem
- Steps to reproduce
- Expected behavior
- Actual behavior
- Environment details (Python version, OS, etc.)

## Questions?

- Check existing issues/PRs
- Review PROTOCOL_DOCUMENTATION.md and MCP_PROTOCOL_NOTES.md
- Open a discussion issue

Thank you for contributing!
