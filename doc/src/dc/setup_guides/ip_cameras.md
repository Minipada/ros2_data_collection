# Ip cameras
To start the ip camera example, to save video feeds from rtsp cameras, you can optionally create a fake video feed if you don't have an ip camera:
0. A [rtsp](https://www.wikiwand.com/en/Real_Time_Streaming_Protocol) server. I use `rtsp-simple-server` available [here](https://github.com/aler9/rtsp-simple-server) and run
```bash
./rtsp-simple-server
```

If the port is already used, download the [configuration](https://github.com/aler9/rtsp-simple-server/blob/main/rtsp-simple-server.yml) and edit the `hlsAddress` parameter. You would then need to start it this way:
```
./rtsp-simple-server conf.yml
```

Send a video feed to the stream with ffmpeg

To send your camera feed, run:
```bash
ffmpeg \
    -f v4l2 \
    -video_size 1920x1080 \
    -i /dev/video2 \
    -f rtsp \
    -rtsp_transport tcp rtsp://127.0.0.1:8554/mystream
```

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

This will create a stream on rtsp://127.0.0.1:8554/mystream

1. Make sure Postgres and MiniIO are set. See [here](./common.md) to know how to start it. Make sure you have set MiniIo credentials properly in your configuration file ip_cameras.yml.

2. Now, start the launch file:
```bash
ros2 launch dc_bringup ip_cameras.launch.py
```
