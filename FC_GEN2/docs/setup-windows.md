# Setting up a Windows device to build and flash the FC_GEN2 project

This document is a basic guide for setting up a Windows device to build and flash the FC_GEN2 project.

## Required Downloads

- [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install)
- [Docker](https://www.docker.com/products/docker-desktop/)
- [CMake](https://cmake.org/download/)
- [Stlink](https://github.com/stlink-org/stlink)

Docker, CMake, and Stlink should be added to your PATH. If not done so automatically by the installer, refer to [this video](https://youtu.be/KbnHT1SoOj0?list=PLEg2mgYz66IOcHRvvUDf9O1ZCGy58M1Bt&t=421) for more information about adding the above tools to your PATH.

## Frequent Issues

Ensure you have updated all your Git submodules. The `build-container` command will do this automatically, but this will also help Intellisense resolve symbols.

```bash
git submodule update --init --recursive
```

The docker daemon must be running for building and flashing, this can be done simply by starting the docker desktop application.