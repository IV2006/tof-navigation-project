#!/bin/bash
# I can learn how to write in bash, or I can write bad code, guess which one is easier

cp /media/sf_AI_extras/*  /home/vboxuser/simulatorMapping/examples

for FILE in /media/sf_AI_extras/*.cpp; do
  BASENAME=$(basename "$FILE" .cpp)
  if ! grep -qw "$BASENAME" "/home/vboxuser/simulatorMapping/examples/CMakeLists.txt"; then
    echo "add_executable($BASENAME $BASENAME.cpp)" >> "/home/vboxuser/simulatorMapping/examples/CMakeLists.txt"
	echo "target_link_libraries($BASENAME simulator ORB_SLAM2 Auxiliary)" >> "/home/vboxuser/simulatorMapping/examples/CMakeLists.txt"
	echo >> "/home/vboxuser/simulatorMapping/examples/CMakeLists.txt"
  fi
done

for FILE in /media/sf_AI_extras/*.cc; do
  BASENAME=$(basename "$FILE" .cc)
  if ! grep -qw "$BASENAME" "/home/vboxuser/simulatorMapping/examples/CMakeLists.txt"; then
    echo "add_executable($BASENAME $BASENAME.cc)" >> "/home/vboxuser/simulatorMapping/examples/CMakeLists.txt"
	echo "target_link_libraries($BASENAME simulator ORB_SLAM2 Auxiliary)" >> "/home/vboxuser/simulatorMapping/examples/CMakeLists.txt"
	echo >> "/home/vboxuser/simulatorMapping/examples/CMakeLists.txt"
  fi
done
