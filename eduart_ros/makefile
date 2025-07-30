image_name = eduart-ros-base:jazzy-1.2.1

build-docker:
	docker build -t $(image_name) .

upload-docker:
	docker push $(image_name)

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

all: build-docker
	$(info Creating EduArt's ROS Jazzy base Docker image "$(image_name)".)
