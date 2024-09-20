image_name = rpliadr:1.2.0

download-sources:
	git clone --branch ros2 https://github.com/Slamtec/rplidar_ros.git
	git clone --branch main https://github.com/Slamtec/sllidar_ros2.git

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

install-udev-rules:
	sudo cp rplidar.rules /etc/udev/rules.d
	sudo service udev reload
	sudo service udev restart

remove-udev-rules:
	sudo rm /etc/udev/rules.d/rplidar.rules
	sudo service udev reload
	sudo service udev restart

clean:
	rm -rf rplidar_ros
	rm -rf sllidar_ros2
