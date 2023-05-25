FROM ubuntu:22.04

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

RUN mkdir /cpp
RUN mkdir /server

WORKDIR /

COPY ./server /server
COPY ./requirements.txt /

RUN pip3 install -r requirements.txt

COPY ./cpp /cpp
COPY ./recastnavigation /recastnavigation

WORKDIR /cpp
RUN make clean_all && make

WORKDIR /

CMD ["python3", "/server/server.py"]
