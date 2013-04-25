ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

include_HEADERS = include/sns.h
pkginclude_HEADERS =                              \
	include/sns/msg.h                         \
	include/sns/daemon.h


lib_LTLIBRARIES = libsns.la libsns_msg_joystick.la

libsns_la_SOURCES = src/msg.c src/daemon.c
libsns_la_LIBADD = -lach


libsns_msg_joystick_la_SOURCES = src/msg/joystick.c
libsns_msg_joystick_la_LIBADD = -lsns

bin_PROGRAMS = snsdump


snsdump_SOURCES = src/snsdump.c
snsdump_LDADD = libsns.la -lach -ldl
