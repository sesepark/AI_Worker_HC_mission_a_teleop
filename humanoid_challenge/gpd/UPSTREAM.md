# GPD Upstream

This directory vendors the upstream GPD source directly in this repository.

- Upstream: https://github.com/atenpas/gpd
- Imported commit: `6327f20eabfcba41a05fdd2e2ba408153dc2e958`
- Local changes: tracked directly in this repository

The Docker image installs GPD build dependencies, but does not copy or build
this source. `humanoid_challenge/docker/container.sh` builds the mounted source
inside the running container so local GPD edits are picked up without rebuilding
the image.
