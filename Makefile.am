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
libsns_la_LIBADD = -lach

## PLUGINS
lib_LTLIBRARIES += libsns_msg_joystick.la
libsns_msg_joystick_la_SOURCES = src/msg_plugin.c
libsns_msg_joystick_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_joystick

lib_LTLIBRARIES += libsns_msg_motor_ref.la
libsns_msg_motor_ref_la_SOURCES = src/msg_plugin.c
libsns_msg_motor_ref_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_motor_ref

lib_LTLIBRARIES += libsns_msg_motor_state.la
libsns_msg_motor_state_la_SOURCES = src/msg_plugin.c
libsns_msg_motor_state_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_motor_state

lib_LTLIBRARIES += libsns_msg_vector.la
libsns_msg_vector_la_SOURCES = src/msg_plugin.c
libsns_msg_vector_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_vector

lib_LTLIBRARIES += libsns_msg_tf.la
libsns_msg_tf_la_SOURCES = src/msg_plugin.c
libsns_msg_tf_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_tf

lib_LTLIBRARIES += libsns_msg_wt_tf.la
libsns_msg_wt_tf_la_SOURCES = src/msg_plugin.c
libsns_msg_wt_tf_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_wt_tf

lib_LTLIBRARIES += libsns_msg_tf_dx.la
libsns_msg_tf_dx_la_SOURCES = src/msg_plugin.c
libsns_msg_tf_dx_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_tf_dx


bin_PROGRAMS = snsdump snsplot sns-trylock


snsdump_SOURCES = src/snsdump.c
snsdump_LDADD = libsns.la -lamino -llapack -lblas

snsplot_SOURCES = src/snsplot.c
snsplot_LDADD = libsns.la -lamino -llapack -lblas

sns_trylock_SOURCES = src/sns-trylock.c


bin_PROGRAMS += snslogd
snslogd_SOURCES = src/snslogd.c
snslogd_LDADD = libsns.la -lamino -llapack -lblas


bin_PROGRAMS += snslog
snslog_SOURCES = src/snslog.c
snslog_LDADD = libsns.la -lamino -llapack -lblas

bin_PROGRAMS += snsref
snsref_SOURCES = src/snsref.c
snsref_LDADD = libsns.la -lamino -llapack -lblas
