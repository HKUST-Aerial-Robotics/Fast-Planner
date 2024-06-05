# Build docker project
.PHONY : docker_build
docker_build:
	docker build -t fastplanner .

# Run docker project
.PHONY : docker_run
docker_run:
	docker run -it  --network host  --entrypoint /bin/bash fastplanner

	