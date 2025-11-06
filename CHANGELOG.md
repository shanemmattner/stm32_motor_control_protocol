# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Modern Python infrastructure with uv
- Code quality tools (black, ruff, mypy)
- Comprehensive testing setup with pytest
- CI/CD automation with GitHub Actions
- Pre-commit hooks for quality assurance
- Serial traffic logs from Motor Pilot reference implementation
- STM32-Motor-Control-Interface submodule
- Comprehensive documentation (INFRASTRUCTURE.md, DEVELOPMENT.md, CONTRIBUTING.md)
- PyPI publishing workflow

### Changed
- Improved README with protocol documentation links

### Fixed

## [0.1.0] - 2025-11-05

### Added
- Initial release of STM32 Motor Control Protocol Python implementation
- MinimalMotorControl class for basic motor control
- Motor control commands (start, stop, mode selection)
- Monitoring commands for real-time data acquisition
- Serial communication via ASPEP protocol
- Example reference implementation
- Protocol documentation extracted from firmware
- MCP protocol notes and specifications

[Unreleased]: https://github.com/shanemmattner/stm32_motor_control_protocol/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/shanemmattner/stm32_motor_control_protocol/releases/tag/v0.1.0
