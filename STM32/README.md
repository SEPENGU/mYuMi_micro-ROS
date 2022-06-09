# micro-ROS for STM32Cube

This guide sets up micro-ROS on STM32CubeIDE, based on the [STM32Cube guide](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/blob/galactic/README.md) by micro-ROS.

Tested with NUCLEO-F767ZI.

## Setup

If you want to create a project from scratch or use another board, follow the [STM32Cube guide](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/blob/galactic/README.md), or the [RTOS guide](https://micro.ros.org/docs/tutorials/core/first_application_rtos/) if your board is [supported](https://micro.ros.org/docs/overview/hardware/).

1. Download and install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Put one of the projects in this folder (STM32) inside the STM32CubeIDE project folder (STM32CubeIDE has to be opened once for it to exist).
3. Clone the Galactic version of the [micro-ROS utilities](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/galactic) at the root of the selected project.
4. Open up STM32CubeIDE and import this project into the workspace.
5. Try building the project. If it doesn't work, continue with the next steps.
6. Go to `Project -> Settings -> C/C++ Build -> Settings -> Build Steps Tab` and in `Pre-build steps` add:

    ```bash
    docker pull microros/micro_ros_static_library_builder:galactic && docker run --rm -v ${workspace_loc:/${ProjName}}:/project --env 
    MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:galactic
    ```

7. Add the micro-ROS include directory to the project: In `Project -> Settings -> C/C++ Build -> Settings -> Tool Settings Tab -> MCU GCC Compiler -> Include paths` add 
`../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include`.

8. Add the micro-ROS precompiled library to the project: In `Project -> Settings -> C/C++ Build -> Settings -> MCU GCC Linker -> Libraries`,
      - add `<ABSOLUTE_PATH_TO>/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros` in `Library search path (-L)`,
      - add `microros` in `Libraries (-l)`.

9. Add the following source code files from `extra_sources/` to `Core/Src/`, replacing the old ones:
      - `microros_time.c`
      - `microros_allocators.c`
      - `custom_memory_manager.c`
      - `microros_transports/dma_transport.c`

10. Build and run the project.
11. Start and connect the [Agent](../Agent.md).

### Troubleshoot

TODO