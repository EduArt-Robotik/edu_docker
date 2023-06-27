image_name = eduartrobotik/eduart-ros-base:humble

build-docker:
	docker build -t $(image_name) .

upload-docker:
	docker push $(image_name)

all: build-docker
	$(info Creating EduArt's ROS Humble base Docker image "$(image_name)".)
