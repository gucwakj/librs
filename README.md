# LibRS

Library for robot simulation engines.  It is split into five separate libraries
which each provide parts of the functionality of the system.

rs		base mathematics classes: position, quaternion, vectors
rsRobots	base robot classes which have dimensions and basic information about
			the robots
rsSim		simulation engine based upon ODE
rsScene		scene graph library based upon OSG
rsCallback	graphical updating library using data from rsSim to manipulate
			rsScene drawings
rsXML		xml parsing library

rsSim, rsScene, and rsXML are independent and each rely upon rs and rsRobots
for basic robot information.  rsCallback is dependent upon rsScene and rsSim.
They can mixed and matched to create a library which provides the simulation
funcionality necessary for specific projects.

## Compilation

Building the librs library and its dependencies is done in four steps.
Compilation is largely based upon CMake build files with an out-of-source build
recommended.  Instructions for Linux, Windows, and the Raspberry Pi are
provided here.  There are three dependency libraries, ODE, OSG, and tinyxml2.
These are built separately and linked to by the librs libraries.  Be default
all modules are built but can be turned off with cmake variables on the command
line.
```
-DENABLE_GRAPHICS=no
-DENABLE_SIM=no
-DENABLE_XML=no
```
These disable support for each of the major components of librs.

### Linux

#### tinyxml2
```
$ cd deps/tinyxml2/
$ mkdir build/
$ cd build/
$ cmake ..
$ make
$ cd ../../../
```

#### open dynamics engine
ODE relies upon configure scripts to build.  The one-liner provided builds the
shared library which is linked in the librs build scripts.  ``make install`` is
used to put all libraries into one folder for easier access.
```
$ cd deps/ode/
$ sh autogen.sh
$ ./configure --prefix=$PWD"/sys/" --enable-double-precision --enable-shared --disable-demos --with-trimesh=none --with-drawstuff=none
$ make
$ make install
$ cd ../../
```

#### openscenegraph
```
$ cd deps/osg/
$ mkdir build/
$ cd build/
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make osg osgDB osgGA osgShadow osgText osgViewer osgdb_png osgdb_3ds
$ cd ../../../
```

#### librs
The four libraries of librs are built into build/lib/.
```
$ mkdir build/
$ cd build/
$ cmake ..
$ make
$ cd ../
```

### Windows

#### tinyxml2
```
cd deps/tinyxml2/
mkdir build/
use cmake-gui to configure with Visual Studio 12 2013 {Win64} and generate
open build/tinyxml2.sln in visual studio
build Release solution
```

#### open dynamics engine
```
cd deps/ode/build/
premake4.exe --only-double --only-shared [--platform=x64] --no-trimesh vs2008
open build/ode.sln in visual studio
build Release solution
```

#### openscenegraph
```
cd deps/osg/
mkdir build/
cd build/
cmake -G "Visual Studio 12 2013 Win64" -DCMAKE_PREFIX_PATH="C:/Qt/5.4.1/5.4/msvc2013_64_opengl/lib/cmake"
	-DDESIRED_QT_VERSION=5 -DACTUAL_3RDPARTY_DIR="../third_party/{x64/x86}" ..
open build/OpenSceneGraph.sln in visual studio
build Release solution
	packages used:	osg osgDB osgFX osgGA osgQt osgShadow osgText
					osgUtil osgViewer osgdb_3ds osgdb_png
```

#### librs
```
mkdir build/
use cmake-gui to configure with visual studio 12 2013 {Win64} and generate
open build/LibRS.sln in visual studio
build Release solution
```

### Raspberry Pi

#### open dynamics engine
ODE relies upon configure scripts to build.  The one-liner provided builds the
shared library which is linked in the librs build scripts.  ``make install`` is
used to put all libraries into one folder for easier access.
```
$ cd deps/ode/
$ sh autogen.sh
$ ./configure --prefix=$PWD"/sys/" --enable-double-precision --enable-shared --disable-demos --disable-asserts --with-trimesh=none --with-drawstuff=none
$ make
$ make install
$ cd ../../
```

#### librs
The default Raspbian image as of August 2015 is based upon Debian Wheezy which
inclues only the gcc 4.6 compiler.  This does not have any support for c++11
which is needed to compile librs.  The newer gcc can be gotten from the Debian
Jessie repositories and specificed on the command line.  Disabling the Graphics
and XML libraries are useful for running headless simulations.  The five
libraries of librs are built into build/lib/.
```
$ mkdir build/
$ cd build/
$ cmake -DCMAKE_C_COMPILER=/usr/bin/gcc-4.9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-4.9 -DENABLE_GRAPHICS=no -DENABLE_XML=no ..
$ make
$ cd ../
```

## Credits
(c) 2014 - 2015

The University of California, Davis
Center for Integrated Computing and STEM Education

