#!/bin/bash

USER=$(whoami)

if [ -d /media/BOOT ]; then
	echo "BOOT mounted in /media"
	BOOTDIR=/media/BOOT/
elif [ -d /media/${USER}/BOOT ]; then
	echo "BOOT mounted in /media/user"
	BOOTDIR=/media/${USER}/BOOT/
elif [ -d /run/media/${USER}/BOOT ]; then
	echo "BOOT mounted in /run/media/user"
	BOOTDIR=/run/media/${USER}/BOOT/
else
	echo "Error: BOOT directory not found or not mounted."
	exit 1
fi

if [ -d /media/SYSTEM ]; then
	echo "SYSTEM mounted in /media"
	LIBDIR=/media/SYSTEM/lib/
elif [ -d /media/${USER}/SYSTEM ]; then
	echo "SYSTEM mounted in /media/user"
	LIBDIR=/media/${USER}/SYSTEM/lib/
elif [ -d /run/media/${USER}/SYSTEM ]; then
	echo "SYSTEM mounted in /run/media/user"
	LIBDIR=/run/media/${USER}/SYSTEM/lib/
else
	echo "Error: SYSTEM directory not found or not mounted."
	exit 1
fi

numArgs=3

if [ "$#" -ne "$numArgs" ]; then
	echo "Error: invalid number of arguments '$# != $numArgs'."
	exit 1
fi

filePath=${1}/${2}-armhf-${3}.tar.gz

if [ ! -f ${filePath} ]; then
	echo "Error: input file '${filePath}' does not exist."
	exit 1
fi

fileSHA256=${1}/${2}-armhf-${3}.sha256

if [ ! -f ${fileSHA256} ]; then
	echo "Error: input file checksum '${fileSHA256}' not found."
	exit 1
fi

fileSum1=$(cat $filePath | sha256sum | head -c 64)
fileSum2=$(cat $fileSHA256 | head -c 64)

if [ ! $fileSum1 = $fileSum2 ]; then
	echo "Error: input file checksum mismatch '${fileSum1} != ${fileSum2}'."
	exit 1
fi

tmpDir=$(mktemp -d)

if [ ! -d ${tmpDir} ]; then
	echo "Error: could not create temp working directory."
	exit 1
else
	echo "Created temp working directory '${tmpDir}'."
fi

function cleanup {
	rm -rf ${tmpDir}
	echo "Deleted temp working directory '${tmpDir}'."
}

trap cleanup EXIT

outputDir=${tmpDir}/output

echo "Decompressing '${filePath}' at '${tmpDir}'..."
tar -xzf ${filePath} -C ${tmpDir} output

if [ -d ${BOOTDIR} ]; then
	sudo rm -f ${BOOTDIR}/*.dtb
	sudo rm -f ${BOOTDIR}/uImage
else
	sudo mkdir ${BOOTDIR}
fi

if [ -d ${LIBDIR} ]; then
	sudo rm -rf ${LIBDIR}/modules
fi

echo "Copying files..."
sudo cp ${outputDir}/uImage ${BOOTDIR}
sudo cp ${outputDir}/*.dtb ${BOOTDIR}
sudo cp -r ${outputDir}/lib/modules ${LIBDIR}
sudo sync

echo "Kernel installed."
exit 0
