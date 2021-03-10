# run from docker

## pull images

```
# pull carlaviz
sudo docker pull selfdriveguard/carlaviz:tagname

# pull 执行引擎
sudo docker pull selfdriveguard/sdg-engine:tagname

# pull sdg-frontend
# sudo docker pull selfdriveguard/sdg-frontend:tagname

# pull sdg-backend
# sudo docker pull selfdriveguard/sdg-backend:tagname

# build carla-autoware
git clone --recurse-submodules git@github.com:SelfDriveGuard/carla-autoware.git
cd carla-autoware && ./build.sh
```

## run

```
# 启动Carla Simulator
./CarlaUE4.sh

# 启动后台
docker run -it --network="host" selfdriveguard/sdg-backend:tagname

# 启动前端
docker run -it --network="host" selfdriveguard/sdg-frontend:tagname

# 启动carlaviz
docker run -it --network="host" -e CARLAVIZ_HOST_IP=localhost -e CARLA_SERVER_IP=localhost -e CARLA_SERVER_PORT=2000 selfdriveguard/carlaviz:tagname

# 启动carla-autoware
# 参照carla-autoware项目中[README.md](https://github.com/SelfDriveGuard/carla-autoware)

# 启动执行引擎
docker run -it --network="host" -e CARLA_SERVER_IP=localhost -i selfdriveguard/sdg-engine:tagname
# 前端访问：http://127.0.0.1:8090/
```