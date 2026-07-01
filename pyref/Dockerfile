# Reproducible host for the pyref/ offline oracle pipeline (REBOUND/ASSIST).
# These are GPL C extensions with no wheels for `assist` and no native-Windows
# support — so they run in this container, never in the shipped Rust binary.
#
# Build:  docker build -t asteroid-pyref pyref/
# Run:    docker run --rm \
#           -v "M:/claud_projects/AsteroidDefense/pyref:/pyref:ro" \
#           -v "<data-cache-dir>:/data" -e ASSIST_DATA_DIR=/data \
#           asteroid-pyref python /pyref/spike_assist_de441.py
#
# The DE ephemeris files (~750 MB) are NOT baked in — they download once into
# the mounted /data cache on first run (see spike_assist_de441.py).
FROM python:3.12-slim

# assist ships no wheel: it compiles from source against REBOUND's headers.
RUN apt-get update -qq \
 && apt-get install -y -qq --no-install-recommends gcc \
 && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir --root-user-action=ignore -r /tmp/requirements.txt

WORKDIR /pyref
CMD ["python", "spike_assist_de441.py"]
