import requests
import os

def main():
    print("开始下载第三方依赖")

    if not os.path.exists("third-party"):
        os.makedirs("third-party")

    url = "https://guard-strike.oss-cn-shanghai.aliyuncs.com/ADTest/carla-0.9.10-py3.7-linux-x86_64.egg"
    path = "third-party/carla-0.9.10-py3.7-linux-x86_64.egg"
    download(url, path)

def download(url, path):
    print("正在下载：{}".format(path))
    the_file = requests.get(url)
    open(path, 'wb').write(the_file.content)


if __name__ == '__main__':
    main()