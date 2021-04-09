# sdg-engine
自驾保执行引擎项目

>本文档是执行引擎项目说明文档，自驾保整体环境搭建，请参照：[环境搭建](https://github.com/SelfDriveGuard/sdg-engine/blob/master/docs/setup/setup.md)

# How to run
1. 安装环境依赖

```
#  安装python3.8以及虚拟环境
sudo apt install python3.8
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.8 5
sudo apt install python3.8-venv

# 安装pip
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py   # 下载安装脚本
sudo apt-get install python3.8-distutils
sudo python get-pip.py
sudo pip install --upgrade keyrings.alt

# 安装poetry
sudo pip install poetry --force 

# 安装第三方依赖
sudo apt install libxerces-c3.2
```

由于poetry实际采用pip进行下载，在不换源的情况下非常缓慢，需要加速

具体参照[常见加速方式一览 五](docs/加速方法.md) 


2. Build and run 

```
poetry install 

poetry shell #进入虚拟环境

python setup.py #下载第三方依赖 

python src/main.py #启动websocket服务

python src/tests/test_engine.py #运行测试
```
