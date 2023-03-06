# Ip cameras
To start the ip camera example, to save video feeds from rtsp cameras, you can optionally create a virtual video feed if you don't have an ip camera, for testing purpose:

## Start RTSP Server
0. A [rtsp](https://www.wikiwand.com/en/Real_Time_Streaming_Protocol) server. I use **rtsp-simple-server** available [here](https://github.com/aler9/rtsp-simple-server). Download from the [release page](https://github.com/aler9/rtsp-simple-server/releases) and start it:

```bash
$ ./rtsp-simple-server
```

If the port is already used, download the [configuration](https://github.com/aler9/rtsp-simple-server/blob/main/rtsp-simple-server.yml) and edit the `hlsAddress` parameter. You would then need to start it this way:

```bash
$ ./rtsp-simple-server conf.yml
```

## Create a virtual camera
This is optional, and only useful if you want to not use a camera

You will first need to install some packages:

```bash
$ sudo apt-get install v4l-utils v4l2loopback
```

And start a virtual camera device:

```bash
$ sudo modprobe v4l2loopback
```

Now check its path:
```bash
$ v4l2-ctl --list-devices

Dummy video device (0x0000) (platform:v4l2loopback-000):
	/dev/video2
```

In my case, it is /dev/video2

To send your camera feed, run:
```bash
ffmpeg \
  -f v4l2 \
  -video_size 1920x1080 \
  -i /dev/video2 \
  -f rtsp \
  -rtsp_transport tcp rtsp://127.0.0.1:8554/mystream
```

## Send video stream

To send an image of a clock ticking, create a blank image and call it bg-white.png (with gimp for example) and run:
```bash
ffmpeg \
  -re -loop 1 \
  -i bg-white.png \
  -vf drawtext="fontfile=monofonto.ttf: fontsize=96: box=1: boxcolor=black@0.75: boxborderw=5: fontcolor=white: x=(w-text_w)/2: y=((h-text_h)/2)+((h-text_h)/4): text='%{gmtime\:%H\\\\\:%M\\\\\:%S}'" \
  -r 25 \
  -vcodec libx264 \
  -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/mystream
```

Now you can use the ip_camera plugin with the url from the command: **rtsp://127.0.0.1:8554/mystream**
