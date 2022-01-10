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
