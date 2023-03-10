##
## This file is part of the libjaylink project.
##
## Copyright (C) 2014-2015 Marc Schink <jaylink-dev@marcschink.de>
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

AC_PREREQ([2.64])

# Package version of libjaylink. This is not the same as the libtool interface
# version.
m4_define([jaylink_version_package_major], [0])
m4_define([jaylink_version_package_minor], [1])
m4_define([jaylink_version_package_micro], [0])
m4_define([jaylink_version_package], [jaylink_version_package_major.jaylink_version_package_minor.jaylink_version_package_micro])

AC_INIT([libjaylink], [jaylink_version_package], [BUG-REPORT-ADDRESS])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_AUX_DIR([build-aux])

AC_CANONICAL_HOST

AM_INIT_AUTOMAKE([foreign -Wall -Werror])

# Enable additional compiler warnings via -Wall and -Wextra. Use hidden
# visibility for all non-static symbols by default with -fvisibility=hidden.
CFLAGS="$CFLAGS -Wall -Wextra -Werror -fvisibility=hidden"

# Checks for programs.
AC_PROG_CC

# Automake >= 1.12 requires AM_PROG_AR when using options -Wall and -Werror.
# To be compatible with older versions of Automake use AM_PROG_AR if it's
# defined only. This line must occur before LT_INIT.
m4_ifdef([AM_PROG_AR], [AM_PROG_AR])

# Initialize libtool.
LT_INIT

# Initialize pkg-config.
PKG_PROG_PKG_CONFIG

# Checks for libraries.

# Check for libusb-1.0 which is always needed.
PKG_CHECK_MODULES([libusb], [libusb-1.0 >= 1.0.9],
	[CFLAGS="$CFLAGS $libusb_CFLAGS"; LIBS="$LIBS $libusb_LIBS"])

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.
AC_C_BIGENDIAN

# Checks for library functions.

# Disable progress and informational output of libtool.
AC_SUBST([AM_LIBTOOLFLAGS], '--silent')

# Libtool interface version of libjaylink. This is not the same as the package
# version. For information about the versioning system of libtool, see:
# http://www.gnu.org/software/libtool/manual/libtool.html#Libtool-versioning
JAYLINK_VERSION_LIB_CURRENT=0
JAYLINK_VERSION_LIB_REVISION=0
JAYLINK_VERSION_LIB_AGE=0
JAYLINK_VERSION_LIB="$JAYLINK_VERSION_LIB_CURRENT:$JAYLINK_VERSION_LIB_REVISION:$JAYLINK_VERSION_LIB_AGE"

AC_SUBST([JAYLINK_VERSION_LIB_CURRENT])
AC_SUBST([JAYLINK_VERSION_LIB_REVISION])
AC_SUBST([JAYLINK_VERSION_LIB_AGE])
AC_SUBST([JAYLINK_VERSION_LIB])

AC_ARG_ENABLE([subproject-build], AS_HELP_STRING([--enable-subproject-build],
	[enable sub-project build [default=no]]))

AM_CONDITIONAL([SUBPROJECT_BUILD],
	[test "x$enable_subproject_build" = "xyes"])

# Libtool interface version is not used for sub-project build as libjaylink is
# built as libtool convenience library.
AS_IF([test "x$enable_subproject_build" != "xyes"],
	[JAYLINK_LIB_LDFLAGS="-version-info $JAYLINK_VERSION_LIB"])

AC_SUBST([JAYLINK_LIB_LDFLAGS])

JAYLINK_VERSION_PACKAGE_MAJOR=jaylink_version_package_major
JAYLINK_VERSION_PACKAGE_MINOR=jaylink_version_package_minor
JAYLINK_VERSION_PACKAGE_MICRO=jaylink_version_package_micro
JAYLINK_VERSION_PACKAGE=jaylink_version_package

AC_SUBST([JAYLINK_VERSION_PACKAGE_MAJOR])
AC_SUBST([JAYLINK_VERSION_PACKAGE_MINOR])
AC_SUBST([JAYLINK_VERSION_PACKAGE_MICRO])
AC_SUBST([JAYLINK_VERSION_PACKAGE])

# Use C99 compatible stdio functions on MinGW instead of the incompatible
# functions provided by Microsoft.
AS_CASE([$host_os], [mingw*],
	[AC_DEFINE([__USE_MINGW_ANSI_STDIO], [1],
		[Define to 1 to use C99 compatible stdio functions on MinGW.])])

# Add the Winsock2 library on MinGW for socket and other network-related
# functions.
AS_CASE([$host_os], [mingw*], [LIBS="$LIBS -lws2_32"])

AC_CONFIG_FILES([Makefile])
AC_CONFIG_FILES([libjaylink/Makefile])
AC_CONFIG_FILES([libjaylink/version.h])
AC_CONFIG_FILES([libjaylink.pc])

AC_OUTPUT

echo
echo "libjaylink configuration summary:"
echo " - Package version ................ $JAYLINK_VERSION_PACKAGE"
echo " - Library version ................ $JAYLINK_VERSION_LIB"
echo " - Installation prefix ............ $prefix"
echo " - Building on .................... $build"
echo " - Building for ................... $host"
echo
