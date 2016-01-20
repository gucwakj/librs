# LibRS

Library for robot simulation engines.  It is split into five separate libraries
which each provide parts of the functionality of the system.

rs			base mathematics classes: position, quaternion, vectors
rsRobots	base robot classes which have dimensions and basic information
			about the robots
rsSim		simulation engine based upon ODE
rsScene		scene graph library based upon OSG
rsCallback	graphical updating library using data from rsSim to manipulate
			rsScene drawings
rsXML		XML parsing library

rsSim, rsScene, and rsXML are independent and each rely upon rs and rsRobots
for basic robot information.  rsCallback is dependent upon rsScene and rsSim.
They can mixed and matched to create a library which provides the simulation
functionality necessary for specific projects.

## Compilation

Building the LibRS library and its dependencies is done in four steps.
Compilation is largely based upon CMake build files with an out-of-source build
recommended.  Instructions for Linux, Windows, and the Raspberry Pi are
provided here.  There are three dependency libraries, ODE, OSG, and tinyxml2.
These are built separately and linked to by the LibRS libraries.  Be default
all modules are built but can be turned off with cmake variables on the command
line.
```
-DRS_GRAPHICS=no
-DRS_SIM=no
-DRS_XML=no
```
These disable support for each of the major components of LibRS.

LibRS can simulate three different types of robot modules.  Each has its own
purpose for learning or researching.  These can also be enabled/disabled
individually to simplify the library for a specific purpose.  By default the
Linkbots and Mindstorms are enabled while the Dof is not.
```
-DRS_DOF=no
-DRS_LINKBOT=no
-DRS_MINDSTORMS=no
```

There are research functions built into LibRS which rely upon the GNU
Scientific Library.  The research pieces of the Dof module can be enabled or
disabled with a CMake definition.
```
-DRS_RESEARCH=yes
```

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
shared library which is linked in the LibRS build scripts.  ``make install`` is
used to put all libraries into one folder for easier access.
```
$ cd deps/ode/
$ sh autogen.sh
$ ./configure --prefix=$PWD"/sys/" --enable-double-precision --enable-shared \
	--with-trimesh=none --with-drawstuff=none \
	--disable-demos --disable-asserts
$ make
$ make install
$ cd ../../
```

#### openscenegraph
```
$ cd deps/osg/
$ mkdir build/
$ cd build/
$ cmake -DDESIRED_QT_VERSION=5 ..
$ make osg osgDB osgGA osgFX osgQt osgShadow osgText osgUtil osgViewer osgdb_png osgdb_3ds
$ cd ../../../
```

#### LibRS
The five libraries of LibRS are built into build/lib/.
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

#### LibRS
```
mkdir build/
use cmake-gui to configure with visual studio 12 2013 {Win64} and generate
open build/LibRS.sln in visual studio
build Release solution
```

### Raspberry Pi

#### Open Dynamics Engine
ODE relies upon configure scripts to build.  The one-liner provided builds the
shared library which is linked in the LibRS build scripts.  ``make install`` is
used to put all libraries into one folder for easier access.
```
$ cd deps/ode/
$ sh autogen.sh
$ ./configure --prefix=$PWD"/sys/" --enable-double-precision --enable-shared \
	--with-trimesh=none --with-drawstuff=none \
	--disable-demos --disable-asserts
$ make
$ make install
$ cd ../../
```

#### LibRS
The default Raspbian image as of August 2015 is based upon Debian Wheezy which
includes only the gcc 4.6 compiler.  This does not have any support for c++11
which is needed to compile LibRS.  The newer gcc can be downloaded from the
Debian Jessie repositories and specified on the command line.  Disabling the
Graphics and XML libraries are useful for running headless simulations.  Also
disabling the unnecessary robots is useful to minimize the size of the library.
The five libraries of LibRS are built into build/lib.
```
$ mkdir build/
$ cd build/
$ cmake -DCMAKE_C_COMPILER=/usr/bin/gcc-4.9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-4.9 \
		-DRS_DOF=yes -DRS_LINKBOT=no -DRS_MINDSTORMS=no \
		-DRS_GRAPHICS=no -DRS_XML=no ..
$ make
$ cd ../
```

### Raspberry Pi 2
The RPi2 has been tested with the Arch Linux ARM distribution.

#### Open Dynamics Engine
ODE relies upon configure scripts to build.  The one-liner provided builds the
shared library which is linked in the LibRS build scripts.  ``make install`` is
used to put all libraries into one folder for easier access.
```
$ cd deps/ode/
$ sh autogen.sh
$ ./configure --prefix=$PWD"/sys/" --enable-single-precision --enable-shared \
	--with-trimesh=none --with-drawstuff=none \
	--disable-demos --disable-asserts
$ make
$ make install
$ cd ../../
```

#### LibRS
Disabling the
Graphics and XML libraries are useful for running headless simulations.  Also
disabling the unnecessary robots is useful to minimize the size of the library.
The five libraries of LibRS are built into build/lib.
```
$ mkdir build/
$ cd build/
$ cmake	-DRS_RESEARCH=yes -DRS_DOF=yes -DRS_LINKBOT=no -DRS_MINDSTORMS=no \
		-DRS_GRAPHICS=no -DRS_XML=no ..
$ make
$ cd ../
```

## Credits
(c) 2014 - 2016

The University of California, Davis
Center for Integrated Computing and STEM Education

