.PHONY: help install install-dev lint format type-check test clean

help:
	@echo "STM32 Motor Control Protocol - Development Commands"
	@echo ""
	@echo "Available commands:"
	@echo "  make install       Install production dependencies with uv"
	@echo "  make install-dev   Install development dependencies with uv"
	@echo "  make lint          Run ruff linter"
	@echo "  make format        Format code with black and isort"
	@echo "  make type-check    Run mypy type checking"
	@echo "  make test          Run pytest tests"
	@echo "  make all           Format, lint, type-check, and test"
	@echo "  make clean         Remove build artifacts and cache files"

install:
	uv sync --no-dev

install-dev:
	uv sync

lint:
	uv run ruff check st_mcp/ tests/ --fix

format:
	uv run black st_mcp/ tests/
	uv run isort st_mcp/ tests/

type-check:
	uv run mypy st_mcp/

test:
	uv run pytest -v --cov=st_mcp

all: format lint type-check test

clean:
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type d -name .pytest_cache -exec rm -rf {} +
	find . -type d -name .mypy_cache -exec rm -rf {} +
	find . -type d -name htmlcov -exec rm -rf {} +
	find . -type f -name .coverage -delete
	rm -rf build/ dist/ *.egg-info

.DEFAULT_GOAL := help
