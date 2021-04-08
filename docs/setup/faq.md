# F.A.Q.

Some of the most common issues regarding SelfDriveGuard installation and builds are listed here.

## runing with docker

<!-- ======================================================================= -->
###### WARNING: Error loading config file: /home/ubuntu/.docker/config.json: open /home/ubuntu/.docker/config.json: permission denied.
> 
> Make sure you have and user to docker group, Open a terminal and run the following commands:
>
>      $ sudo groupadd docker
>      $ sudo gpasswd -a ${USER} docker
>      $ newgrp docker
>      $ sudo chown "$USER":"$USER" /home/"$USER"/.docker -R
>      $ sudo chmod g+rwx "/home/$USER/.docker" -R
>      $ sudo systemctl enable docker
>      $ sudo systemctl restart docker
>      $ docker version //看到版本信息即可
> 

<!-- ======================================================================= -->
###### blocked at [Wait]Initializing Ego.... in sdg-engine
> 
> Check folder size of autoware-contents in carla-autoware, it must be 2.6G. [download](https://guard-strike.oss-cn-shanghai.aliyuncs.com/ADTest/autoware-contents.tar.gz) and replace
> 