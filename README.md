# LLCC68 driver

This package proposes an implementation in C of the driver for **LLCC68** radio component.
Please see the [changelog](CHANGELOG.md) for more information.

## Structure

The driver is defined as follows:

- llcc68.c: implementation of the driver functions
- llcc68.h: declarations of the driver functions
- llcc68_regs.h: definitions of all useful registers (address and fields)
- llcc68_hal.h: declarations of the HAL functions (to be implemented by the user - see below)

## HAL

The HAL (Hardware Abstraction Layer) is a collection of functions the user shall implement to write platform-dependant calls to the host. The list of functions is the following:

- llcc68_hal_reset
- llcc68_hal_wakeup
- llcc68_hal_write
- llcc68_hal_read

## Cmake usage

This driver exposes a cmake configuration allowing to integrate the driver in a cmake ready application.

### Integration

If the driver code resides in a directory of the application using it, it can be integrated by adding the subdirectory to the configuration as follows:

```cmake
add_subdirectory(llcc68_driver) # where llcc68_driver is the name of the folder containing the driver code
```

Alternatively, if the driver code is available through a code archive, it can be included directly by

```cmake
include(FetchContent)
FetchContent_Declare(
    llcc68_driver
    URL "path_to_archive" # Where path_to_archive is to be replaced by the path to the archive driver
)
FetchContent_MakeAvailable(llcc68_driver)
```

