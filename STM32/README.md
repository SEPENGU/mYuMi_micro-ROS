# micro-ROS for STM32Cube

This guide sets up micro-ROS on STM32CubeIDE using Micro XRCE-DDS as the middleware, and is based on the [official guide](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/blob/galactic/README.md) by micro-ROS.

## Setup

1. Download [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Put this folder (STM32) inside the STM32CubeIDE project folder (STM32CubeIDE has to be opened once for it to exist).
3. Clone the Galactic version of the [STM32Cube utilities](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/galactic) at the root of this folder.
4. Open up STM32CubeIDE and import this project into the workspace.
5. Go to `Project -> Settings -> C/C++ Build -> Settings -> Build Steps Tab` and in `Pre-build steps` add:

```bash
docker pull microros/micro_ros_static_library_builder:galactic && docker run --rm -v ${workspace_loc:/${ProjName}}:/project --env 
MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:galactic
```

6. Add micro-ROS include directory. In `Project -> Settings -> C/C++ Build -> Settings -> Tool Settings Tab -> MCU GCC Compiler -> Include paths` add 
`micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include`

7. Add the micro-ROS precompiled library. In `Project -> Settings -> C/C++ Build -> Settings -> MCU GCC Linker -> Libraries`
      - add `<ABSOLUTE_PATH_TO>/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros` in `Library search path (-L)`
      - add `microros` in `Libraries (-l)`
8. Build and run your project.

### Troubleshoot
