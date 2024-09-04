#!/usr/bin/env bash

echo -e "Building adaptive3dp:lastest image"

DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ./.docker/Dockerfile \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--target zsh \
--tag adaptive3dp:latest .