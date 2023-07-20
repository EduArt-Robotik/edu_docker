image_name = rpliadr:1.0.0

download-sources:
	git clone --branch ros2 https://github.com/babakhani/rplidar_ros2.git

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

install-udev-rules:
	sudo rplidar_ros2/scripts/create_udev_rules.sh

remove-udev-rules:
	sudo rplidar_ros2/scripts/delete_udev_rules.sh

clean:
	rm -rf rplidar_ros2
