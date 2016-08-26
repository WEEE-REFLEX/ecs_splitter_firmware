Install instructions for OpenSTM32 System Workbench aka SW4STM (installed with stand alone installer)
I won't follow automatic procedure of SW4STM with auto-download of HAL drivers in order to understand what's going on under the hood
I will get MCU headers and sources from examples of X-Nucleo-Spn3 as well as SW4STM configuration files
Unfortunately, manually setting includes, linked resources, startup files and linker scripts results in a real mess.

The structure will be like this:
- ecs_splitter_firmware folder (main, that is under version control): contains only source&headers that are project-specific
- STM32CubeExpansion-SPN3_V1.2.0: as downloaded by ST; we will use only shield-specific files from here*

*HAL drivers can also come from STM32Cube_FW_F4. We are using those of STM32CubeExpansion-SPN3_V1.2.0 since they are already available.
Please notice that they can differ.

