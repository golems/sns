ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

include_HEADERS = include/sns.h
pkginclude_HEADERS =                              \
	include/sns/msg.h                         \
	include/sns/util.h                        \
	include/sns/daemon.h


init_ddir = $(sysconfdir)/init.d
init_d_SCRIPTS = scripts/sns

lib_LTLIBRARIES = libsns.la
lib_LTLIBRARIES += libsns_msg_joystick.la
lib_LTLIBRARIES += libsns_msg_motor_ref.la
lib_LTLIBRARIES += libsns_msg_motor_state.la

libsns_la_SOURCES = src/msg.c src/daemon.c src/util.c
libsns_la_LIBADD = -lach -ldl -lamino


libsns_msg_joystick_la_SOURCES = src/msg/joystick.c
libsns_msg_joystick_la_LIBADD = libsns.la

libsns_msg_motor_ref_la_SOURCES = src/msg/motor_ref.c
libsns_msg_motor_ref_la_LIBADD = libsns.la

libsns_msg_motor_state_la_SOURCES = src/msg/motor_state.c
libsns_msg_motor_state_la_LIBADD = libsns.la

bin_PROGRAMS = snsdump snsplot sns-trylock


snsdump_SOURCES = src/snsdump.c
snsdump_LDADD = libsns.la -lach -lamino

snsplot_SOURCES = src/snsplot.c
snsplot_LDADD = libsns.la -lach -lamino

sns_trylock_SOURCES = src/sns-trylock.c

bin_PROGRAMS += snslogd
snslogd_SOURCES = src/snslogd.c
snslogd_LDADD = libsns.la -lach -lamino

bin_PROGRAMS += snslog
snslog_SOURCES = src/snslog.c
snslog_LDADD = libsns.la -lach -lamino
