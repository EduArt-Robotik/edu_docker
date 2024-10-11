image_name = sensor:1.0.0

download-sources:
	git clone --branch ros2 https://github.com/Slamtec/rplidar_ros.git
	git clone --branch main https://github.com/Slamtec/sllidar_ros2.git
	git clone --branch master https://github.com/SICKAG/sick_lidar_localization.git

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

clean:
	rm -rf rplidar_ros
	rm -rf sllidar_ros2
	rm -rf sick_lidar_localization
