# labrador-linux-6
Linux 6 kernel source code for Caninos Labrador Single Board Computer.

## About
This repository contains the source code of Caninos Labrador's linux
kernel. It supports both 32-bit (Labrador Core V2.x) and 64-bit 
(Labrador Core V3.x) architectures.

## Clean Build
Prior to compilation, make sure you have the following libraries and/or
tools installed on your machine:
1) GNU Binutils environment and Native GCC compiler
2) GCC cross-compiler targeting "gcc-aarch64-linux-gnu"
3) GCC cross-compiler targeting "gcc-arm-linux-gnueabihf"
4) Make build tool
5) Git client
6) Bison and Flex development libraries
7) NCurses development libraries
8) LibSSL development libraries
9) U-Boot Tools libraries

```
$ sudo apt-get update
$ sudo apt-get install build-essential binutils make git
$ sudo apt-get install gcc-arm-linux-gnueabihf gcc-aarch64-linux-gnu
$ sudo apt-get install bison flex libncurses-dev libssl-dev u-boot-tools
```

After installing these tools, clone this repository in your computer.
Then, go to it's main directory and execute it's makefile.

```
$ git clone https://github.com/caninos-loucos/labrador-linux-6.git
$ cd labrador-linux-6
$ make all
$ make all32
```

## Incremental Build (64-bit)
If you want to do an incremental build, execute the following commands:

1) To load 64-bit configuration
```
make config
```
>Note: this will overwrite all configurations already set up.

2) To change which modules are compiled into your kernel image
```
make menuconfig
```
3) To compile the device tree binary blob
```
make dtbs
```
4) To compile your kernel image
```
make kernel
```
5) To reset everything
```
make clean
```

## Incremental Build (32-bit)
If you want to do an incremental build, execute the following commands:

1) To load 32-bit configuration
```
make config32
```
>Note: this will overwrite all configurations already set up.

2) To change which modules are compiled into your kernel image
```
make menuconfig32
```
3) To compile the device tree binary blob
```
make dtbs32
```
4) To compile your kernel image
```
make kernel32
```
5) To reset everything
```
make clean32
```

## Kernel

After a successful compilation, the kernel should be available in 
the "output" folder or in the "output32" folder, for 64-bit and 32-bit 
architectures, respectively.

## Contributing

**Caninos Loucos Forum: <https://forum.caninosloucos.org.br/>**
**Caninos Loucos Website: <https://caninosloucos.org/>**

