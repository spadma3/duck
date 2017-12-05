#!/bin/sh

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

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/yunfantang/duckietown/catkin_ws/src/30-localization-and-planning/duckietown_description"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/yunfantang/duckietown/catkin_ws/install_isolated/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/yunfantang/duckietown/catkin_ws/install_isolated/lib/python2.7/dist-packages:/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_description/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_description" \
    "/usr/bin/python" \
    "/home/yunfantang/duckietown/catkin_ws/src/30-localization-and-planning/duckietown_description/setup.py" \
    build --build-base "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_description" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/yunfantang/duckietown/catkin_ws/install_isolated" --install-scripts="/home/yunfantang/duckietown/catkin_ws/install_isolated/bin"
