image_name = eduart-vnc-ros:0.1.0

upload-docker:
	docker push $(image_name)

build-local:
	docker buildx build --platform linux/amd64 -t eduartrobotik/$(image_name) --load .

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t eduartrobotik/$(image_name) --push .
