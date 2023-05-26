FROM debian:bullseye

RUN apt update -y
RUN apt install -y \
    make \
    cmake \
    build-essential \
    clang \
    python3 \
    python3-pip

# build dependencies
RUN apt install -y \
    libglm-dev \
    freeglut3 \
    freeglut3-dev \
    binutils-gold \
    libsdl2-dev

RUN mkdir /app
RUN mkdir /app/cpp
RUN mkdir /app/server

WORKDIR /app

COPY ./server /app/server
COPY ./requirements.txt /app

RUN pip3 install -r requirements.txt

COPY ./cpp /app/cpp
COPY ./recastnavigation /app/recastnavigation

WORKDIR /app/cpp
RUN make clean_all && make

WORKDIR /app

CMD ["python3", "/app/server/server.py"]
