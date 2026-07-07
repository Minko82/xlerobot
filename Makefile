.PHONY: install install-dev lint format test smoke docker-build

PYTHON ?= python3

# Editable install with all robot/teleop extras (see pyproject.toml).
install:
	$(PYTHON) -m pip install -e ".[all]"

# Install plus developer tooling (linting, tests).
install-dev: install
	$(PYTHON) -m pip install pre-commit pytest
	pre-commit install

lint:
	pre-commit run --all-files

# Hardware-free checks: syntax-compile the codebase and verify model assets.
smoke:
	$(PYTHON) -m compileall -q src examples diagnostics set_motor_id.py
	$(PYTHON) -m pytest tests -q

test: smoke

docker-build:
	docker build -f docker/Dockerfile.user -t xlerobot-pro .
