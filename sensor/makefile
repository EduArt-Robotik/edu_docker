image_name = sensor:1.0.1

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

clean:
	rm -rf rplidar_ros
	rm -rf sllidar_ros2
	rm -rf sick_lidar_localization
	rm -rf leuze_rsl_ros2_drivers
