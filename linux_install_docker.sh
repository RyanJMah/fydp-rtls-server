set -e

# install docker
sudo apt update

sudo apt install apt-transport-https ca-certificates curl software-properties-common
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu focal stable"
sudo apt-cache policy docker-ce

sudo apt install docker-ce

# configure to use docker without sudo
sudo groupadd docker
sudo usermod -aG docker $USER

