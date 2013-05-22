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

libsns_la_SOURCES = src/msg.c src/daemon.c src/util.c
libsns_la_LIBADD = -lach -ldl -lamino


lib_LTLIBRARIES += libsns_msg_joystick.la
libsns_msg_joystick_la_SOURCES = src/msg/joystick.c
libsns_msg_joystick_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_motor_ref.la
libsns_msg_motor_ref_la_SOURCES = src/msg/motor_ref.c

lib_LTLIBRARIES += libsns_msg_motor_state.la
libsns_msg_motor_state_la_SOURCES = src/msg/motor_state.c

lib_LTLIBRARIES += libsns_msg_vector.la
libsns_msg_vector_la_SOURCES = src/msg/vector.c

bin_PROGRAMS = snsdump snsplot sns-trylock


snsdump_SOURCES = src/snsdump.c
snsdump_LDADD = libsns.la -lach -lamino -lblas -llapack

snsplot_SOURCES = src/snsplot.c
snsplot_LDADD = libsns.la -lach -lamino -lblas -llapack

sns_trylock_SOURCES = src/sns-trylock.c


bin_PROGRAMS += snslogd
snslogd_SOURCES = src/snslogd.c
snslogd_LDADD = libsns.la -lach -lamino -lblas -llapack


bin_PROGRAMS += snslog
snslog_SOURCES = src/snslog.c
snslog_LDADD = libsns.la -lach -lamino -lblas -llapack

bin_PROGRAMS += snsref
snsref_SOURCES = src/snsref.c
snsref_LDADD = libsns.la -lach -lamino -lblas -llapack
