# Camera
To save image frames, have a camera connected, or set up a fake one.

0. To set up a fake one, run:

```bash
sudo apt install v4l2loopback
sudo modprobe v4l2loopback
```

This will create a new device in /dev name /dev/video2 (or 3 or 4...)

Run ffmpeg to send the feed:

To send an image of a clock ticking, create a blank image and call it bg-white.png (with gimp for example) and run:
```bash
ffmpeg \
    -re -loop 1 \
    -i bg-white.png \
    -vf drawtext="fontfile=monofonto.ttf: fontsize=96: box=1: boxcolor=black@0.75: boxborderw=5: fontcolor=white: x=(w-text_w)/2: y=((h-text_h)/2)+((h-text_h)/4): text='%{gmtime\:%H\\\\\:%M\\\\\:%S}'" \
    -r 25 \
    -vcodec libx264 \
    -f v4l2 /dev/video4
```

Change the /dev/video4 by your device name

1. 
