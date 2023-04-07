#FROM ubuntu:20.04
FROM nvidia/cuda:11.1.1-runtime-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
RUN apt-get -y update
RUN apt-get install -y tzdata

RUN apt-get -y update
RUN apt-get -y install wget unzip 

RUN apt-get -y update
RUN apt-get -y install git cmake g++ make

RUN apt-get -y update
RUN apt-get -y install libgl1-mesa-dev libgl1-mesa-glx libosmesa6-dev patchelf libopenmpi-dev libglew-dev libglfw3 libglfw3-dev

# for display through vnc
RUN apt-get -y update
#RUN DEBIAN_FRONTEND=noninteractive apt-get install -y xfce4 tigervnc-standalone-server #원격조정을 위한 명령어==필요없음

RUN apt-get update
RUN apt-get -y install curl
RUN apt-get -y install python3-venv
RUN apt install -y python3-pip
RUN pip3 install ipython

# drake
RUN apt-get update
RUN curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-focal.tar.gz
RUN mkdir -p /python_venvs/tamp
RUN tar -xvzf drake.tar.gz -C /python_venvs/tamp --strip-components=1
RUN yes | /python_venvs/tamp/share/drake/setup/install_prereqs
WORKDIR /python_venvs/
RUN python3 -m venv tamp --system-site-packages
WORKDIR /python_venvs/tamp

# ompl
RUN wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh -O /install-ompl-ubuntu.sh
RUN bash /install-ompl-ubuntu.sh --python

# utilities 
RUN apt-get update
RUN apt-get -y install firefox
RUN apt-get -y install vim
RUN apt-get -y install tmux
RUN apt-get -y install openssh-server
RUN apt-get -y install sudo
RUN apt-get -y install xfce4-terminal
RUN apt-get -y install gedit
RUN apt-get -y install trash-cli
RUN apt-get -y install ffmpeg
RUN apt-get -y install xdg-utils
RUN apt-get -y install liboctomap-dev libfcl-dev

RUN apt-get -y install apt-transport-https gnupg
RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > bazel.gpg
RUN mv bazel.gpg /etc/apt/trusted.gpg.d/ 
RUN echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
RUN apt-get update
RUN apt-get -y install bazel
RUN apt-get -y install jq

# pddlstream
#RUN git clone --recurse-submodules https://github.com/caelan/pddlstream.git pddlstream
#RUN cd pddlstream && ./FastDownward/build.py release64
#RUN cd pddlstream/FastDownward/builds && ln -s release64 release32
#ENV PYTHONPATH=${PYTHONPATH}:/python_venvs/tamp/pddlstream

# update terminal config
RUN printf "2\n" | update-alternatives --config x-terminal-emulator

# arguments when building image
# login password 
ARG password 
# $UID
ARG id
# $USER
ARG user
ENV PASS=$password

RUN groupadd -g $id $user
RUN useradd -s /bin/bash -u $id -g $id -m $user && echo "$user:$password" | chpasswd && adduser $user sudo
RUN chown -R $user /python_venvs 
ENV PATH="/python_venvs/tamp/bin:$PATH"
USER $user

WORKDIR /home/$user/
# mujoco
RUN wget https://www.roboti.us/download/mujoco200_linux.zip -O mujoco.zip
RUN unzip mujoco.zip -d .mujoco
RUN mv .mujoco/mujoco200_linux .mujoco/mujoco200
RUN wget https://www.roboti.us/file/mjkey.txt -O /home/$user/.mujoco/mjkey.txt
# add mujoco to LD_LIBRARY_PATH
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$user/.mujoco/mujoco200/bin

WORKDIR /home/$user/
# start ikea specific stuff #
RUN git clone https://github.com/BenAgro314/furniture.git
WORKDIR /home/$user/furniture
RUN git fetch origin aug2020_fixes:aug2020_fixes
RUN git checkout aug2020_fixes
RUN ln -s /home/$user/.mujoco/mujoco200 /home/$user/.mujoco/mujoco200_linux
#RUN pip install --upgrade pip
RUN pip3 install scikit-build
RUN pip3 install glfw>=1.4.0 numpy>=1.19.5 Cython>=0.27.2 imageio>=2.1.2 cffi>=1.10 fasteners~=0.15 lockfile
# copy mujoco license key `mjkey.txt` 
#COPY mjkey.txt /home/$user/.mujoco/mjkey.txt
#RUN pip3 install -r requirements.txt

RUN pip3 install backcall==0.1.0
RUN pip3 install certifi==2019.9.11
RUN pip3 install cffi==1.13.2
RUN pip3 install chardet==3.0.4
RUN pip3 install cloudpickle==1.2.2
RUN pip3 install colorlog==4.0.2
RUN pip3 install Cython==0.29.14
RUN pip3 install decorator==4.4.1
RUN pip3 install entrypoints==0.3
RUN pip3 install future==0.18.2
RUN pip3 install glfw==1.8.4
RUN pip3 install gym==0.15.4
RUN pip3 install idna==2.8
RUN pip3 install imageio==2.6.1
RUN pip3 install imageio-ffmpeg==0.3.0
RUN pip3 install ipdb==0.12.2
RUN pip3 install ipython==7.9.0
RUN pip3 install ipython-genutils==0.2.0
RUN pip3 install lockfile==0.12.2
RUN pip3 install mccabe==0.6.1
RUN pip3 install moviepy==1.0.1
RUN pip3 install mpi4py==3.0.3
RUN pip3 install mujoco-py==2.0.2.5
RUN pip3 install numpy==1.17.4
RUN pip3 install opencv-python==4.1.2.30
RUN pip3 install pexpect==4.7.0
RUN pip3 install pickleshare==0.7.5
RUN pip3 install Pillow==6.2.1
RUN pip3 install proglog==0.1.9
RUN pip3 install prompt-toolkit==2.0.10
RUN pip3 install ptyprocess==0.6.0
RUN pip3 install pybullet==2.5.7
RUN pip3 install pygame==1.9.6
RUN pip3 install pyglet==1.3.2
RUN pip3 install Pygments==2.4.2
RUN pip3 install PyOpenGL==3.1.0
RUN pip3 install pyquaternion==0.9.5
RUN pip3 install requests==2.22.0
RUN pip3 install scipy==1.3.2
RUN pip3 install six==1.13.0
RUN pip3 install tqdm==4.38.0
RUN pip3 install traitlets==4.3.3
RUN pip3 install urllib3==1.25.7
RUN pip3 install wcwidth==0.1.7

WORKDIR /home/$user/
# other requirements
#COPY requirements.txt requirements.txt
#RUN pip3 install -r requirements.txt

#RUN pip3 install numpy==1.19.5
#RUN pip3 install numpy-stl
#RUN pip3 install pybullet
#RUN pip3 install scipy
#RUN pip3 install pandas
#RUN pip3 install open3d==0.10.0.0
#RUN pip3 install black
#RUN pip3 install pyvis
#RUN pip3 install pympler
#RUN pip3 install psutil
#RUN pip3 install python-fcl
#RUN pip3 install ipykernel
#RUN pip3 install tikzplotlib

RUN pip3 install torch==1.9.0+cu111 -f https://download.pytorch.org/whl/torch_stable.html
RUN pip install torch-scatter torch-sparse torch-cluster torch-spline-conv torch-geometric -f https://pytorch-geometric.com/whl/torch-1.9.0+cu111.html
ENV PYTHONPATH=${PYTHONPATH}:/home/$user/drake-tamp/:/home/$user/drake-tamp/pddlstream/:/home/$user/furniture/:/home/$user/drake-tamp/experiments/ikea_induction/

RUN echo "source /python_venvs/tamp/bin/activate" >> ~/.bashrc
RUN echo "export  USER=$user" >> ~/.bashrc
RUN echo "export  PYTHONPATH=${PYTHONPATH}:/home/$user/drake-tamp/:/home/$user/drake-tamp/pddlstream/:/home/$user/experiments/ikea_induction/:/home/$user/drake-tamp/pddlstream/examples/pybullet/utils/" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$user/.mujoco/mujoco200/bin" >> ~/.bashrc

ARG gitname 
ARG gitemail 

RUN git config --global user.name $gitname
RUN git config --global user.email $gitemail

WORKDIR /home/$user/drake-tamp
COPY .tmux.conf /home/$user/.tmux.conf
COPY .vimrc /home/$user/.vimrc

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["bash","/entrypoint.sh"]

CMD ["bash"]
