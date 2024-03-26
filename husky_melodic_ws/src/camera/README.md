# install archiconda
go to: https://github.com/Archiconda/build-tools/releases
download: Archiconda3-0.2.3-Linux-aarch64.sh
cd download
sh ./Archiconda3-0.2.3-Linux-aarch64.sh


# configure conda environment
conda create -n camera python=3.6

conda activate camera

# install pytorch
wget https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl -O
torch-1.8.0-cp36-cp36m-linux_aarch64.whl
sudo apt-get install python3-pip libopenblas-base libopenmpi-dev
pip3 install Cython
pip3 install numpy torch-1.8.0-cp36-cp36m-linux_aarch64.whl
# degrade numpy since 1.19.5 was automatically installed and it is an abomination!!!!!
pip3 isntall numpy==1.19.4



# install torchvision
sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch v0.9.0 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.9.0
python3 setup.py install

pip3 install pillow (if you faced syntax error when installing pillow)


# install open-cv

# install ros-cv-bridge
