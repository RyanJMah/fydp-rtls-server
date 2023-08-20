FROM debian:bullseye

RUN apt update -y
RUN apt install -y \
    make \
    cmake \
    build-essential \
    python3 \
    python3-pip \
    libglm-dev \
    freeglut3 \
    freeglut3-dev \
    binutils-gold \
    libsdl2-dev \
    mosquitto \
    supervisor

RUN mkdir -p /app/cpp
RUN mkdir -p /app/server
RUN mkdir -p /app/resources
RUN mkdir -p /app/logs

ADD ./supervisord.conf /etc/supervisor/conf.d/

WORKDIR /app

COPY ./requirements.txt /app
RUN pip3 install -r requirements.txt

COPY ./cpp /app/cpp
COPY ./recastnavigation /app/recastnavigation

WORKDIR /app/cpp
RUN make clean_all && make

ADD ./mosquitto.conf /app
ADD ./gl_conf.json /app

COPY ./resources /app/resources
COPY ./server /app/server
COPY ./tests /app/tests

WORKDIR /app

EXPOSE 1883

CMD ["supervisord", "-n"]
