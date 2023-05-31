FROM debian:bullseye

RUN apt update -y
RUN apt install -y \
    make \
    cmake \
    build-essential \
    python3 \
    python3-pip

# build dependencies
RUN apt install -y \
    libglm-dev \
    freeglut3 \
    freeglut3-dev \
    binutils-gold \
    libsdl2-dev

RUN mkdir -p /app/cpp
RUN mkdir -p /app/server
RUN mkdir -p /app/resources

WORKDIR /app

COPY ./requirements.txt /app
RUN pip3 install -r requirements.txt

COPY ./resources /app/resources
COPY ./server /app/server
COPY ./cpp /app/cpp
COPY ./recastnavigation /app/recastnavigation

WORKDIR /app/cpp
RUN make clean_all && make

WORKDIR /app

CMD ["python3", "/app/server/server.py"]
