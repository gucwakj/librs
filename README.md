# LibRS

Library for robot simulation engines.  It is split into four separate libraries
which each provide parts of the functionality of the system.

rsRobots	base robot classes which have dimensions and basic information about
			the robots
rsSim		simulation engine based upon ODE
rsScene		scene graph library based upon OSG
rsXML		xml parsing library

rsSim, rsScene, and rsXML are independent and each rely upon rsRobots for basic
robot information.  They can mixed and matched to create a library which
provides the simulation funcionality necessary for specific projects.

## Compilation

Building the librs library and its dependencies is done in four steps.
Compilation is largely based upon CMake build files with an out-of-source build
recommended.  Instructions for Linux and Windows are provided here.

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
$ make osg osgDB osgGA osgShadow osgText osgViewer osgdb_png osgdb_3ds osgdb_tga
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
use cmake-gui to configure with visual studio 9 and generate
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
use cmake-gui to configure with visual studio 9 and generate
	config option: ACTUAL_3RDPARTY_DIR set to osg/third_party/VC9/{x86/x64}
open build/OpenSceneGraph.sln in visual studio
build Release solution, unecessary plugins can be disabled
	curently used plugins:
		3ds
		tga
		png
```

#### librs
```
mkdir build/
use cmake-gui to configure with visual studio 9 and generate
open build/robosim.sln in visual studio
build Release solution
```

## Credits
(c) 2014

The University of California, Davis
Center for Integrated Computing and STEM Education

