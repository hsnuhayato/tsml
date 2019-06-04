#!/bin/bash
xhost +si:localuser:root

docker run  -it --name choreonoid_sandbox --rm   \
       -v $(pwd):/mnt --workdir=/mnt \
       --net host \
       --privileged     \
       --env="DISPLAY"  \
       --env="QT_X11_NO_MITSHM=1" \
       --env="LANG=ja_JP.UTF-8" \
       hsnuhayato/choreonoid1.7 /bin/bash
# /usr/local/bin/choreonoid

