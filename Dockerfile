FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-z3 \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir control

WORKDIR /app

COPY *.py ./

ENTRYPOINT ["python3"]
CMD ["patcher-1.py"]  # Default script if no argument is provided
