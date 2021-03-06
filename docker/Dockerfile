# The version of Python is python3.8
FROM ubuntu:20.04

RUN useradd -ms /bin/bash sdg

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \ 
  apt-get install -y gcc make build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev libncursesw5-dev xz-utils tk-dev libffi-dev liblzma-dev tcptraceroute bc net-tools

RUN apt update && \
  apt install -y python3-pip && \
  pip --version

RUN pip install --upgrade "requests==2.25.1" && \
  pip install --upgrade "websockets==8.1" && \
  pip install --upgrade "numpy==1.19" && \
  pip install --upgrade "networkx==2.5" && \
  pip install --upgrade "antlr4-python3-runtime==4.8.0" && \
  pip install --upgrade "roslibpy==1.1.0" && \
  pip install --upgrade "metric-temporal-logic==0.2.2" && \
  pip install --upgrade "scenic==2.0.0" && \
  pip install --upgrade "tabulate==0.7.5" && \
  pip install Shapely && \
  pip install flask

RUN apt update && \
  apt install -y git libxerces-c3.2 libjpeg8 libtiff5 iputils-ping

# clone source code
RUN cd /home/sdg/ && \
  git clone git://github.com/SelfDriveGuard/sdg-engine.git && \
  cd /home/sdg/sdg-engine/ && \
  /usr/bin/python3.8 setup.py 

COPY run.sh /home/sdg/

RUN chmod -R 777 /home/sdg/

USER sdg

ENV CARLA_SERVER_IP localhost

WORKDIR /home/sdg

CMD ["./run.sh"]

ENTRYPOINT ["/bin/bash", "-c"]