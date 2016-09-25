#!/bin/sh
#author: Clarence Chan, Sept 23th, 2016

#set environment
sudo apt-get install g++ libX11-dev libXext-dev libXtst-dev libqglviewer-dev
sudo apt-get update

#install Qt4.8
tar zxvf qt-everywhere-opensource-src-4.8.7.tar.gz -C ~/Package
cd ~/Package
cd qt-everywhere-opensource-src-4.8.7
./configure -multimedia -opengl -fontconfig
make
sudo make install
sed -i -e '$a QTDIR=/usr/local/Trolltech/Qt-4.8.7/' -e '$a PATH=$QTDIR/bin:$PATH' -e '$a MANPATH=$QTDIR/doc/man:$MANPATH' -e '$a LD_LIBRARY_PATH=$QTDIR/lib:$LD_LIBRARY_PATH' -e '$a export QTDIR PATH MANPATH LD_LIBRARY_PATH' ~/.bashrc
echo "**************************************"
echo "Finished! Now you can install QtCreator."



