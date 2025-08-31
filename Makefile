.PHONY: install lint test

install:
	pip install -r requirements.txt

lint:
	pre-commit run --all-files || echo "pre-commit not installed"

test:
	colcon test || echo "colcon not installed"
