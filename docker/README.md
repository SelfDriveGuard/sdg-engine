
# sdg-engine
这是sdg-engine引擎的docker文档。
镜像基于Ubuntu20.04和python3.7.9，预安装了sdg-engine项目的各种依赖。

#### How to Pull from Dockerhub and Run it

1. Run following command to pull this image.

   ```sh
   docker pull selfdriveguard/sdg-engine:[version tag]
   ```

2. Run following command to run this image.

   ```sh
   docker run -it --network="host" selfdriveguard/sdg-engine:[version tag]
   ```