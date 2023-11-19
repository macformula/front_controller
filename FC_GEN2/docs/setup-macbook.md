# Setting up a Macbook(M1/M2) to build and flash the FC_GEN2 project

This document is a basic guide for setting up a Mac device to build and flash the FC_GEN2 project. 

## Downloads

- [Docker](https://www.docker.com/products/docker-desktop/) *Optional*
- [CMake](https://cmake.org/download/)
- [GCC Arm](https://developer.arm.com/downloads/-/gnu-rm)

GCC Arm and CMake should be added to your PATH. 

## Changes

1. Makefile: remove initial for loop that sets up "Build_System" variable 
2. Makefile: Set “BUILD_SYSTEM ?= Unix Makefiles” 
3. CMakeLists.txt: Delete all references of "-Wformat-truncatio"
4. CMakeLists.txt: Delete all references of "--print-memory-usage" 
5. Ensure you have updated all your Git submodules

```bash
git submodule update --init --recursive
```

** make build-container currently not possible - working to get docker instructions for Mac (so currently not needed) 
