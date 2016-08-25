Install instructions for OpenSTM32 System Workbench aka SW4STM (installed with stand alone installer)
I won't follow automatic procedure with auto-download of HAL drivers in order to understand what's going on under the hood
I will get MCU headers and sources from examples of X-Nucleo-Spn3, but not IDE-specific configuration files
I will manually set Includes, Build Variables and so on...

The structure will be like this:
- ecs_splitter_firmware folder (main, that is under version control): contains only source&headers that are project-specific
- STM32CubeExpansion-SPN3_V1.2.0: as downloaded by ST; we will use only shield-specific files from here*
- STM32Cube_FW_F4_V1.13.0: as downloaded by ST; we will use drivers files from here*

*HAL drivers can come from X-Nucleo-Spn3 xor from STM32Cube_FW_F4. I let the user decide which set has to be used.

