import requests
import os
import sys

def main():
    print("开始下载第三方依赖")

    if not os.path.exists("third-party"):
        os.makedirs("third-party")

    url = "https://guard-strike.oss-cn-shanghai.aliyuncs.com/ADTest/carla-0.9.10-py3.7-linux-x86_64.egg"
    path = "third-party/carla-0.9.10-py3.7-linux-x86_64.egg"
    download(url, path)

    url = "https://guard-strike.oss-cn-shanghai.aliyuncs.com/ADTest/scenic.tar.gz"
    path = "third-party/scenic.tar.gz"
    download(url, path)
    unzip(path)
    delete(path)

def download(url, path):
    with open(path, "wb") as f:
        print("正在下载：{}".format(path))
        response = requests.get(url, stream=True)
        total_length = response.headers.get('content-length')

        if total_length is None: # no content length header
            f.write(response.content)
        else:
            dl = 0
            total_length = int(total_length)
            for data in response.iter_content(chunk_size=4096):
                dl += len(data)
                f.write(data)
                done = int(50 * dl / total_length)
                sys.stdout.write("\r[%s%s]" % ('=' * done, ' ' * (50-done)) )    
                sys.stdout.flush()
        print("")

def unzip(path):
    print("正在解压缩:{}".format(path))
    os.system("tar -xzf {} -C third-party/".format(path))

def delete(path):
    os.system("rm {}".format(path))


if __name__ == '__main__':
    main()