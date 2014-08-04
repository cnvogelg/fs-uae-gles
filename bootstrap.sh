#!/bin/sh -e
echo "Boostrapping fs-uae..."
# version number is cached in autom4te.cache
rm -Rf autom4te.cache
# cat m4/* | grep -v "#serial" > acinclude.m4
echo "Running aclocal -I m4"
if [ -e /opt/share/aclocal ]; then
	# extra m4 macros installed on mingw system
	aclocal -I m4 -I /opt/share/aclocal
else
	aclocal -I m4
fi
echo "Running autoheader"
autoheader
echo "Running automake --add-missing --copy"
automake --add-missing --copy
echo "Running autoconf"
autoconf
echo "Bootstrap done, you can now run ./configure"
