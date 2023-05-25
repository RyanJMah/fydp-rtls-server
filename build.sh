IMAGE_NAME="rtls_server_debian_bullseye"
CONTAINER_NAME="rtls_server"

set -e

function clean_docker() {
    set +e
    docker rm $CONTAINER_NAME &> /dev/null
    docker rmi $IMAGE_NAME &> /dev/null
    set -e
}

if [[ $1 == "clean" ]]
then
    clean_docker
    exit 0
fi

if [[ $1 == "clean_all" ]]
then
    clean_docker
    cd cpp && clean_all
    exit 0
fi

if [[ $1 == "cpp_local" ]]
then
    cd cpp && make
    exit 0
fi

# build docker image and container

set +e
docker stop $CONTAINER_NAME &> /dev/null
docker rm $CONTAINER_NAME &> /dev/null
set -e

docker build -t $IMAGE_NAME .
docker container create -it --name $CONTAINER_NAME $IMAGE_NAME

if [[ $1 == "start" ]]
then
    docker start $CONTAINER_NAME
fi
