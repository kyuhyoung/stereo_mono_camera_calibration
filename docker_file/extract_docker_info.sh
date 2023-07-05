#!/bin/bash
echo "=================================================================================================================="
echo "Docker container info."
echo "=================================================================================================================="
echo "USAGE :"
echo " docker build --force-rm -t \${docker_name} -f docker_file/Dockerfile_\${docker_name}"
echo " docker run -w \$PWD -v \$PWD:\$PWD \${docker_name} sh -c \". ~/.bashrc && . docker_file/extract_docker_info.sh\""
echo "------------------------------------------------------------------------------------------------------------------"
os=$(cat /etc/os-release | grep 'PRETTY_NAME' | cut -d '=' -f 2 | cut -d '"' -f 2) 
echo "OS : ${os}"
cpp=$(g++ --version 2>/dev/null | grep 'g++' | rev | cut -d' ' -f1 | rev)
echo "c++ : ${cpp}"
#: << 'END'
python=$(python3 --version | xargs | rev | cut -d' ' -f1 | rev)
echo "python : ${python}" 
#cuda=$(ls -l /usr/local/ | grep 'cuda ->' | rev | cut -d'-' -f1 | rev)
#cuda=$(cat /usr/local/cuda/version.txt)
#cuda=$(nvcc -V | grep 'release' | rev | cut -d' ' -f1 | cut -d'V' -f1 | rev)
cuda=$(ls -l /usr/local | grep "cuda ->" | rev | cut -d' ' -f1 | rev | xargs realpath | rev | cut -d'-' -f1 | rev)
#cuda=$(realpath /usr/local/cuda | rev | cut -d'-' -f1 | rev)
echo "cuda : ${cuda}"
cudnn1=$(find /usr/lib/x86_64-linux-gnu -name "libcudnn.so.*.*" | rev | cut -d'.' -f3 | rev)
cudnn2=$(find /usr/lib/x86_64-linux-gnu -name "libcudnn.so.*.*" | rev | cut -d'.' -f2 | rev)
cudnn3=$(find /usr/lib/x86_64-linux-gnu -name "libcudnn.so.*.*" | rev | cut -d'.' -f1 | rev)
echo "cudnn : ${cudnn1}.${cudnn2}.${cudnn3}"
#cv=$(pip3 list | grep opencv | xargs | rev | cut -d' ' -f1 | rev) 
cv=$(/usr/local/bin/opencv_version) 
echo "opencv : ${cv}"
tf=$(pip3 list | grep "tensorflow " | xargs | rev | cut -d' ' -f1 | rev)
echo "tensorflow : ${tf}"
torch=$(pip3 list | grep "torch " | xargs | rev | cut -d' ' -f1 | rev)
echo "pytorch : ${torch}"
tv=$(pip3 list | grep torchvision | xargs | rev | cut -d' ' -f1 | rev)
echo "torchvison : ${tv}"
paddle=$(pip3 list | grep paddlepaddle | xargs | rev | cut -d' ' -f1 | rev)
echo "paddlepaddle : ${paddle}"
trt=$(pip3 list | grep "tensorrt " | xargs | rev | cut -d' ' -f1 | rev) 
echo "tensorRT : ${trt}"
tb=$(pip3 list | grep "tensorboard " | xargs | rev | cut -d' ' -f1 | rev)
echo "tensorboard : ${tb}"
bazel=$(bazel version 2>/dev/null | grep "label:" | xargs | rev | cut -d' ' -f1 | rev)
echo "bazel : ${bazel}"
echo "=================================================================================================================="

#END
