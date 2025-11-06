# Modern Python Infrastructure Setup

This document describes the modern Python tooling and best practices implemented in this project.

## Package Management: uv

**uv** is a fast, modern Python package manager written in Rust. Benefits:

- **10-100x faster** than pip
- Deterministic dependency resolution
- Single binary installation
- Lock file support for reproducibility
- Project-level tool management

### Quick Start

```bash
# Install uv (once)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Clone and setup project
git clone --recurse-submodules git@github.com:shanemmattner/stm32_motor_control_protocol.git
cd stm32_motor_control_protocol

# Install dependencies
uv sync
```

## Project Configuration: pyproject.toml

Modern Python projects use `pyproject.toml` for all configuration (PEP 517/518 compliant).

**Our setup includes:**
- **Build system**: hatchling
- **Dependencies**: pyserial, numpy
- **Development tools**: pytest, black, ruff, mypy, isort
- **Documentation**: sphinx-rtd-theme
- **Tool configurations**: black, ruff, mypy, pytest

### Adding Dependencies

```bash
# Add production dependency
uv add pyserial

# Add development dependency
uv add --dev pytest

# Sync environment
uv sync
```

## Code Quality Tools

### Black - Code Formatting

Opinionated code formatter for consistent style.

```bash
make format  # or: uv run black st_mcp/ tests/
```

Config: 100 character lines, Python 3.8+

### Ruff - Fast Linter

Blazingly fast Python linter (written in Rust).

```bash
make lint    # or: uv run ruff check st_mcp/ tests/ --fix
```

Checks:
- E/W: pycodestyle (PEP 8)
- F: pyflakes
- I: isort (import ordering)
- C: flake8-comprehensions
- B: flake8-bugbear

### isort - Import Sorting

Sorts imports consistently.

```bash
uv run isort st_mcp/ tests/
```

Config: black-compatible profile

### mypy - Type Checking

Static type checker for Python.

```bash
make type-check  # or: uv run mypy st_mcp/
```

Ensures type hints are correct and catches type-related bugs.

### pytest - Testing Framework

Testing framework with coverage support.

```bash
make test    # or: uv run pytest -v --cov=st_mcp
```

Features:
- Parametrized tests
- Fixtures
- Coverage reporting
- HTML coverage reports (htmlcov/)

## Development Workflow

### Common Tasks

```bash
# Install dependencies
make install           # Production only
make install-dev       # With dev tools

# Development checks
make format            # Format code
make lint              # Lint code
make type-check        # Type check
make test              # Run tests
make all               # Format + lint + type-check + test

# Cleanup
make clean             # Remove build artifacts
```

### Development Loop

1. **Make changes**: Edit code
2. **Format**: `make format` (auto-fixes style)
3. **Lint**: `make lint` (auto-fixes issues)
4. **Type-check**: `make type-check` (catch errors)
5. **Test**: `make test` (verify functionality)
6. **Commit**: `git commit`

## CI/CD: GitHub Actions

Automated testing on every push and PR.

### Workflow: `.github/workflows/test.yml`

**Runs on:**
- Python 3.8, 3.9, 3.10, 3.11, 3.12
- Ubuntu latest
- Every push to main/develop
- Every pull request

**Steps:**
1. Checkout code
2. Setup Python
3. Install uv
4. Install dependencies
5. Lint (ruff)
6. Format check (black)
7. Type check (mypy)
8. Run tests (pytest with coverage)
9. Upload to Codecov

### Badge

Add to README:
```markdown
[![Tests](https://github.com/shanemmattner/stm32_motor_control_protocol/actions/workflows/test.yml/badge.svg)](https://github.com/shanemmattner/stm32_motor_control_protocol/actions)
```

## Pre-commit Hooks

Automatically run checks before commits.

### Setup

```bash
uv add --dev pre-commit
uv run pre-commit install
```

### Configuration: `.pre-commit-config.yaml`

Hooks run on every commit:
- black: Format code
- isort: Sort imports
- ruff: Lint
- bandit: Security checks
- YAML/JSON validation

### Manual Run

```bash
# Run all hooks on all files
uv run pre-commit run --all-files

# Update hooks to latest versions
uv run pre-commit autoupdate
```

## Security: Bandit

Scans Python code for security issues.

Config: `.bandit` - Skips test assertions (B101)

Checks for:
- SQL injection
- Hardcoded passwords
- Insecure random
- Subprocess usage
- Shell injection

## Version Management

### `.python-version`

Specifies project Python version (3.11).

With `pyenv`:
```bash
pyenv install 3.11
pyenv local 3.11  # Creates .python-version
```

## Editor Configuration

### `.editorconfig`

Enforces consistent coding style across editors:
- UTF-8 encoding
- LF line endings
- 4-space indentation (Python)
- 100 character max line length
- Trim trailing whitespace

Supported editors: VSCode, PyCharm, Sublime, Vim, etc.

## Makefile

Convenience commands for common tasks.

```bash
make help         # Show all available commands
make install      # Install production dependencies
make install-dev  # Install development dependencies
make lint         # Run linter with auto-fix
make format       # Format code
make type-check   # Run type checker
make test         # Run tests with coverage
make all          # Format + lint + type-check + test
make clean        # Clean build artifacts
```

## Package Structure

```
st_mcp/
├── __init__.py              # Package exports
├── minimal_motor_control.py # Example implementation
└── commands/
    ├── __init__.py          # Command exports
    ├── motor.py             # Motor commands
    └── monitor.py           # Monitor commands

tests/
├── __init__.py
└── test_example.py          # Example tests
```

## Testing Best Practices

### Test Organization

```python
class TestMotorCommand:
    """Group related tests in classes."""

    def test_specific_behavior(self):
        """Clear test names describe what is being tested."""
        # Arrange
        cmd = StartMotorCommand(motor_id=0)

        # Act
        data = cmd.to_bytes()

        # Assert
        assert len(data) >= 4
```

### Coverage

- Target: 80%+ coverage
- Run: `make test` (generates htmlcov/index.html)
- View: Open `htmlcov/index.html` in browser

## Dependencies

### Production
- **pyserial** >= 3.5: Serial communication
- **numpy** >= 1.21.0: Numerical operations

### Development
- **pytest** >= 7.0: Testing framework
- **pytest-cov** >= 4.0: Coverage reporting
- **black** >= 23.0: Code formatting
- **ruff** >= 0.1.0: Linting
- **mypy** >= 1.0: Type checking
- **isort** >= 5.12: Import sorting

### Optional
- **sphinx** >= 5.0: Documentation
- **sphinx-rtd-theme** >= 1.0: ReadTheDocs theme

## Troubleshooting

### uv not found

Install uv:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Python version mismatch

Check your Python version:
```bash
python --version
```

Set globally or per-project:
```bash
pyenv local 3.11
```

### Dependency conflicts

Clear and reinstall:
```bash
uv sync --refresh
```

### Pre-commit hook issues

Update hooks:
```bash
uv run pre-commit autoupdate
```

## Resources

- [uv Documentation](https://github.com/astral-sh/uv)
- [PEP 517/518 (pyproject.toml)](https://peps.python.org/pep-0517/)
- [Black Code Style](https://black.readthedocs.io/)
- [Ruff Documentation](https://docs.astral.sh/ruff/)
- [mypy Documentation](https://mypy.readthedocs.io/)
- [pytest Documentation](https://docs.pytest.org/)
