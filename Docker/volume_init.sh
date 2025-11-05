#!/usr/bin/env bash
set -e
docker compose run --rm thesis bash -lc "volume_init.sh"
