作者：Clarence Chan     日期：2015.8.6
e-mail:chg10086@yeah.net

注意：配置及代码适用环境：ubuntu14.04
ubuntu14.10等版本应该也可以，但是作者并没有试过，某些路径可能略有不同

源码下载:https://github.com/g-ch/Clarence-Chan
可以先在系统中安装git，终端执行 sudo apt-get install git
然后在某个目录中（默认home下）执行
git clone https://github.com/g-ch/Clarence-Chan.git
或者 git clone git@github.com:g-ch/Clarence-Chan.git


源码编译需要安装的环境有：
qt4.8.7+qtcreator3.1.2(推荐搭配，注意qt只能用qt4系列，不能用qt5)
ros -jade系统及mavros

/*************第一步-安装qt相关环境******************/

qt所有相关内容下载网址：
http://download.qt.io
需要下载qt4.8.7的源码（qt-everywhere-opensource-src-4.8.7.tar.gz）以及qtcreator3.1.2.run文件

一、按装qt4.8.7
1、需安装几个环境
终端运行
sudo apt-get install g++ libX11-dev libXext-dev libXtst-dev
sudo apt-get install libqglviewer-dev
sudo apt-get update
2、解压tar.gz包
tar zxvf xxx.tar.gz
此处xxx为下载的qt-everywhere-opensource-src-4.8.7

4、进入qt4.8.7安装源码的目录
./configure -multimedia -opengl
 (in case qtopengl is not installded)tar 
choose opensource 

接着就make，然后sudo make install就可以了
（make这里时间比较长，可以先去吃饭睡觉打豆豆了）


5、设置环境：

当这几个步骤全部完成后
执行vi ~/.bashrc 添加如下内容：
QTDIR=/usr/local/Trolltech/Qt-4.8.7/
PATH=$QTDIR/bin:$PATH
MANPATH=$QTDIR/doc/man:$MANPATH
LD_LIBRARY_PATH=$QTDIR/lib:$LD_LIBRARY_PATH
export QTDIR PATH MANPATH LD_LIBRARY_PATH
之后重启。如果不会用vi可以执行vi语句后输入:$回车，然后输入o进入输入模式，将上面的字段拷贝后按esc，然后输入大写的ZZ或者输入：wq即可。



二、qt creator的安装：
从官网下载下来qt creator，(qt-creator-linux-x86_64-opensource-3.1.2.run或更高版本），如果该文件没有可执行权限的话要先把权限改了：

chmod a+x qt-creator-linux-x86_64-opensource-3.1.2.run

然后执行：sudo ./qt-creator-linux-x86_64-opensource-3.1.2.run

就会然让你选个位置来安装，这里会自动关联到qt4.8.7中，安装好即ok

三、运行
使用终端打开qtcreator安装目录下bin文件夹中的qtcreator即可。
如：终端执行 cd ~/qtcreator-3.1.2/bin
            ./qtcreator

进入后可以现自己新建一个工程编译试试，但是之后需要用的编译器不是qt用的qmake，而是cmake，所以之后需要打开工程，详细见后面的教程。

/******************第二步-安装ros相关环境**************************/
ros系统官网：ros.org
ubuntu安装jade版本的ros网址http://wiki.ros.org/jade/Installation/Ubuntu

一、安装ros jade（内容来自官网教程的翻译）

1、终端输入
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 设置环境

2、设定密钥
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

3、确保所有环境为最新版本
终端输入sudo apt-get update

4、安装ros jade
sudo apt-get install ros-jade-desktop-full
（默认安装到opt/下）

5、初始化rosdep
sudo rosdep init
rosdep update

6、设置ros环境，这样就不用每次都要setup一下了～
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc
或者直接用vi把source /opt/ros/jade/setup.bash添加到~/.bashrc中也可以
如果仅要配置一次环境的话，source /opt/ros/jade/setup.bash

7、安装rosinstall，为了方便以后获取别的包，还是装一下吧
sudo apt-get install python-rosinstall

8、安装mavros（参见http://wiki.ros.org/mavros）

原版mavros安装方法如下：
终端执行
sudo apt-get install ros-jade-mavros ros-jade-mavros-extras

*如果是从github上下载的源码安装为ros工作空间下的一个包（在下一步建立完工作空间之后完成），需要将路径/catkin_ws/src/mavros/libmavconn下的CMakeLists.txt中的catkin_add_gtest(mavconn-test test/test_mavconn.cpp)改为add_library(mavconn-test test/test_mavconn.cpp)，也可以自行在ros上配置gtest的环境使用，这里为了方便就直接替换吧。
*如果是从github上下载的源码安装为ros工作空间下的一个包,需要自己安装一下mavlink
git clone https://github.com/mavlink/mavlink-gbp-release.git -b debian/indigo/trusty/mavlink
cd mavlink-gpb-release
sed 's|\(2014.11.11-3trusty\)|\1-custom|' -i debian/changelog
双击安装deb文件
fakeroot dpkg-buildpackage -us -uc -b


/**********************************************************/
                  华丽丽的分割线
到这里需要联网安装的环境就结束了，下面就来建立自己的开发区域吧
/**********************************************************/


/*****************第三步-建立自己的ros工作空间*******************/
以下内容参见：http://wiki.ros.org/ROS/Tutorials
首先，之前没有把source /opt/ros/jade/setup.bash添加到~/.bashrc中的先在终端执行一次source /opt/ros/jade/setup.bash配置一下ros的环境

然后在home下创建一个名字叫做catkin_ws的文件夹作为工作空间并初始化，执行
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

现在可以试着编译一下，执行
cd ~/catkin_ws/
catkin_make
其中catkin_make是一个很好用的编译工具，不过我们后面不会通过这种方式编译，直接在qt中编译就可以了。这里只要不报错就可以了，报错呢就google一下吧，肯定不是教程的问题。。
catkin_make之后生成的文件默认保存在catkin_ws/build/下

然后再setup一下工作空间：
source devel/setup.bash 
这样工作空间的搭建就完成了，如果需要检验以下的话可以在终端输入：
echo $ROS_PACKAGE_PATH
你应该会看到显示
/home/youruser/catkin_ws/src:/opt/ros/jade/share:/opt/ros/jade/stacks

/********************第四步-建立包*********************/
这里就简单介绍一下怎么建立就好了
1、打开工作空间目录
cd ~/catkin_ws/src
2、建立一个包
catkin_create_pkg (name)  
（name）为名字，这里为了保持一致性，不用再去修改CMakeList.txt,就把名字写为 station
即执行 catkin_create_pkg station 

以下第3步是用于给自己新建的包改的内容，从git上下载下来的源码已经包含，所以可以直接跳过
3、package.xml下license修改为BSD，然后在catkin_ws目录下catkin_make

4、把从github下载的源码copy到这个包里，git clone下来的文件应该有一个名字叫做station的文件夹，将里面的所有内容复制，粘贴到刚刚建立的~/catkin_ws/src/station 下，替换原来的文件。
*如果你前面创建的包的名字不叫station，那么需要打开~/catkin_ws/src/(name)/CMakeList.txt,把里面所有的station替换为你的命名，之后的操作凡是遇到station也同样全部改为你自己命的名字

5、编译一下看看
cd ~/catkin_ws
catkin_make
如果不报错就ok了，可以转往qt了

/******************第五步-在qt中运行**********************/
终于到了最后一步了！！
1、首先打开qtcreator
如：终端执行 cd ~/qtcreator-3.1.2/bin
            ./qtcreator

2、选择openproject，选ros工作空间对应的包的CMakeList.txt，即~/catkin_ws/src/station文件夹下CMakeList.txt,注意不要选成~/catkin_ws/src下和station文件夹并列的那个CMakeList.txt；

3、Browse 将路径修改为 ~/catkin_ws/build/，点击确定

4、Arguments 一栏填入 -DCMAKE_BUILD_TYPE=Debug

5、点击run cmake,等待编译完后点击finish
如果不报错就ok了

6、为了使图片保存的路径正常，需要修改camera.h文件下第17行的#define  system_user_name "chg" 中的"chg"为自己的系统用户名，如果忘记了点ubuntu系统右上角的设置按钮即可看到当前登录的用户名，ubuntu为了用户的安全，home每个文件中间把用户名作为路径添加了进去，以防止访客看到其中的文件，所以这里需要设置这一步

接下来每次修改完代码后只需要用build一下然后运行即可


