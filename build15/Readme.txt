This is a manually prepared solution for Visual Studio 2015.

Compared to CMake generated one, it has the projects categorized
as main / examples. It also has legacy x86 builds. It will not
regenerate the workspace upon CMakeList.txt changes, so that it
will never erase e.g. your commandline arguments settings.

Other than that, CMake is now quite safe to use.
