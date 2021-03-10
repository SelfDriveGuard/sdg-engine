# run from code

## clone && compile

```
# 后台
git clone https://github.com/SelfDriveGuard/sdg-backend.git

# 前端
git clone https://github.com/SelfDriveGuard/sdg-frontend.git

# 执行引擎
git clone https://github.com/SelfDriveGuard/sdg-engine.git

# Carlaviz后端
git clone https://github.com/SelfDriveGuard/carlaviz.git

#Carla-autoware
git clone --recurse-submodules https://github.com/SelfDriveGuard/carla-autoware.git
#Clone结束后，根据各项目目录下README.md进行编译配置

```

## run

```
#启动carla模拟器
cd CARLA_0.9.10/
./CarlaUE4.sh

# 启动carlaviz后端
cd carlaviz
./backend/bin/backend

# 启动后台
cd sdg-backend/
node app.js

# 启动前端页面
cd sdg-frontend/
npm start
浏览器打开: http://localhost:8090/

# 启动carla-autoware
# 参照carla-autoware项目中[README.md](https://github.com/SelfDriveGuard/carla-autoware)

# 启动执行引擎
cd sdg-engine/
poetry shell
python src/main.py #启动
```
