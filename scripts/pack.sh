#!/bin/bash

numArgs=5

if [ "$#" -ne "$numArgs" ]; then
	echo "Error: invalid number of arguments '$# != $numArgs'"
	exit 1
fi

if [ ! -d "$1" ]; then
	echo "Error: build directory '$1' does not exist"
	exit 1
fi

mach=${3^^}

case $mach in
	AARCH64 | ARMHF)
	;;
	*)
	echo "Error: invalid machine type '${mach}'"
	exit 1
esac

buildDir=$1
releaseDir=$2
tmpDir=$(mktemp -d)

if [ ! -d "$tmpDir" ]; then
	echo "Error: could not create temp working directory."
	exit 1
else
	echo "Created temp working directory '${tmpDir}'"
fi

function cleanup {
	rm -rf "$tmpDir"
	echo "Deleted temp working directory '${tmpDir}'"
}

trap cleanup EXIT

outputDir="${tmpDir}"/output
libDir="${outputDir}"/lib

mkdir "${outputDir}"

if [ ! -d "${outputDir}" ]; then
	echo "Error: output directory '${outputDir}' does not exist"
	exit 1
else
	echo "Created temp output directory '${outputDir}'"
fi

mkdir "${libDir}"

if [ ! -d "${libDir}" ]; then
	echo "Error: lib directory '${libDir}' does not exist"
	exit 1
else
	echo "Created temp lib directory '${libDir}'"
fi

if [ "$mach" = "AARCH64" ]; then
	name=${4}-aarch64-${5}.tar.gz
	nameSHA256=${4}-aarch64-${5}.sha256
	cp "${buildDir}"/arch/arm64/boot/dts/caninos/*.dtb "${outputDir}"
	cp "${buildDir}"/arch/arm64/boot/dts/caninos/k7-base21.dtb "${outputDir}"/v3psci.dtb
	cp "${buildDir}"/arch/arm64/boot/Image "${outputDir}"
	cp -rf "${buildDir}"/lib/modules "${libDir}"
else
	name=${4}-armhf-${5}.tar.gz
	nameSHA256=${4}-armhf-${5}.sha256
	cp "${buildDir}"/arch/arm/boot/dts/caninos-k5.dtb "${outputDir}"/kernel.dtb
	cp "${buildDir}"/arch/arm/boot/uImage "${outputDir}"
	cp -rf "${buildDir}"/lib/modules "${libDir}"
fi

find "${libDir}" -type l -exec rm -f {} \;
cd ${tmpDir}
tar -czf ${name} output
sha256sum ${name} > ${nameSHA256}
mkdir -p ${releaseDir}
cp ${name} ${releaseDir}
cp ${nameSHA256} ${releaseDir}
cd $OLDPWD
exit 0

