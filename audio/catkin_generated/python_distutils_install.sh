#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/marco/catkin_ws/src/athomesoftware/audio"

# todo --install-layout=deb per platform
# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/marco/catkin_ws/install/lib/python2.7/dist-packages:/home/marco/catkin_ws/src/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/marco/catkin_ws/src" \
    "/usr/bin/python" \
    "/home/marco/catkin_ws/src/athomesoftware/audio/setup.py" \
    build --build-base "/home/marco/catkin_ws/src/athomesoftware/audio" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/marco/catkin_ws/install" --install-scripts="/home/marco/catkin_ws/install/bin"
