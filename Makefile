.PHONY: install lint build test

install:
	pip install -r requirements.txt

lint:
	pre-commit run --all-files

build:
	cd ros_ws && colcon build --symlink-install --event-handlers console_direct+

test:
	cd ros_ws && colcon test --return-code-on-test-failure --event-handlers console_direct+
	cd ros_ws && colcon test-result --verbose
