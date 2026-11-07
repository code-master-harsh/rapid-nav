# Development

## Building without a native ROS install

Use this path on a Mac or on any machine without ROS 2 Jazzy. The build runs in Docker; on Ubuntu 24.04 build
natively instead (see [setup.md](setup.md)).

- Docker Desktop: start it first.
- `scripts/dev.sh image`: build the image `rapid-nav-dev:jazzy` from `docker/Dockerfile`.
- `scripts/dev.sh build`: run `colcon build` on `src/`.
- `scripts/dev.sh test`: run `colcon test` on `src/` and print the test results.
- `scripts/dev.sh deps`: check the package dependencies with rosdep.
- `scripts/dev.sh run '<command>'`: run one command with ROS and the workspace sourced.
- `scripts/dev.sh shell`: open an interactive bash with the same mounts.
- `scripts/dev.sh clean`: delete the Docker volume that holds the build.

The build happens in a Docker volume, so the repository's own `build/` and `install/` are not touched.

## Limits

The image renders sensors in software. Rates are lower than on a native machine, and there is no GUI.

## Code style

Python follows `ament_flake8` and `ament_pep257` (run by `colcon test`). XML is checked by `ament_xmllint` where
packages enable it.
