#!/bin/bash
# Activation script for pixi environments.
# Sources the colcon-generated setup file if it exists.
# On a clean checkout this is a no-op; run `pixi run build` first.

# Detect the multiarch tuple for the current CPU (works on x86_64 and aarch64).
_ARCH="$(uname -m)"
_MULTIARCH="${_ARCH}-linux-gnu"

# The conda-provided pkg-config binary auto-rewrites 'prefix' by going two
# directory levels up from the .pc file location.  For multiarch system paths
# like /usr/lib/x86_64-linux-gnu/pkgconfig/ this yields the wrong prefix
# (/usr/lib instead of /usr), breaking cmake find_package calls.
# We ship per-arch .pc files under pkgconfig/<arch>/ with hardcoded absolute
# paths.  Pick the right directory based on the detected CPU.
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PKG_CONFIG_PATH="${_SCRIPT_DIR}/pkgconfig/${_ARCH}${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
unset _SCRIPT_DIR

# The conda GCC uses a minimal sysroot that does not contain system-installed
# package headers.  Symlink any required system headers into the conda env's
# include dir, which the compiler searches via -idirafter.
# libserial is the only system-only C++ library used here.
if [ -d "/usr/include/libserial" ] && [ ! -e "${CONDA_PREFIX}/include/libserial" ]; then
    ln -sf /usr/include/libserial "${CONDA_PREFIX}/include/libserial"
fi

# The conda sysroot ships old CRT startup files (Scrt1.o etc.) that still
# reference __libc_csu_init / __libc_csu_fini.  Those symbols were removed in
# glibc 2.34, so they are absent on Ubuntu 22.04+ (glibc 2.35) and the linker
# fails when building any executable.  Replace the sysroot CRT files with
# symlinks to the system versions, which are glibc 2.35-compatible.
# The original files are preserved as *.conda_backup.
_CONDA_TRIPLE="${_ARCH}-conda-linux-gnu"
_SYSROOT_LIBDIR="${CONDA_PREFIX}/${_CONDA_TRIPLE}/sysroot/usr/lib64"
_SYS_LIBDIR="/usr/lib/${_MULTIARCH}"
for _crt in Scrt1.o crt1.o crti.o crtn.o; do
    if [ -f "${_SYS_LIBDIR}/${_crt}" ] && [ ! -L "${_SYSROOT_LIBDIR}/${_crt}" ]; then
        mv "${_SYSROOT_LIBDIR}/${_crt}" "${_SYSROOT_LIBDIR}/${_crt}.conda_backup" 2>/dev/null || true
        ln -sf "${_SYS_LIBDIR}/${_crt}" "${_SYSROOT_LIBDIR}/${_crt}"
    fi
done
unset _ARCH _MULTIARCH _CONDA_TRIPLE _SYSROOT_LIBDIR _SYS_LIBDIR _crt

if [ -f "install/setup.sh" ]; then
    source install/setup.sh
fi
