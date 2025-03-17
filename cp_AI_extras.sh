#!/bin/sh

shared='/media/sf_AI_extras'
simulator=/home/vboxuser/simulatorMapping
copy_if_different() {
    src="$1"
    dest="$2"
    if [ ! -f "$dest" ] || ! cmp -s "$src" "$dest"; then
        echo "Copying: $src -> $dest"
        cp "$src" "$dest"
    fi
}

copy_if_different "$shared/Auxiliary.h" "$simulator/include/Auxiliary.h"
copy_if_different "$shared/Auxiliary.cpp" "$simulator/src/Auxiliary.cpp"
copy_if_different "$shared/RoomExit.h" "$simulator/include/RoomExit/RoomExit.h"
copy_if_different "$shared/RoomExit.cpp" "$simulator/src/RoomExit/RoomExit.cpp"
copy_if_different "$shared/Navigator.h" "$simulator/include/Navigator/Navigator.h"
copy_if_different "$shared/Navigator.cpp" "$simulator/src/Navigator/Navigator.cpp"
copy_if_different "$shared/main.cpp" "$simulator/examples/main.cpp"
copy_if_different "$shared/generalSettings.json" "$simulator/generalSettings.json"
