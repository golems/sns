ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

include_HEADERS = include/sns.h
pkginclude_HEADERS =                              \
	include/sns/msg.h                         \
	include/sns/util.h                        \
	include/sns/daemon.h			   \
	include/sns/event.h			   \
	include/sns/path.h                         \
        include/sns/sdh_tactile.h


init_ddir = $(sysconfdir)/init.d
init_d_SCRIPTS = scripts/sns

lib_LTLIBRARIES = libsns.la
libsns_la_SOURCES = src/msg.c src/daemon.c src/util.c src/msg/path.c src/event.c
libsns_la_LIBADD = -lach -lamino -lach-experimental

## PLUGINS
lib_LTLIBRARIES += libsns_msg_joystick.la
libsns_msg_joystick_la_SOURCES = src/msg_plugin.c
libsns_msg_joystick_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_joystick
libsns_msg_joystick_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_motor_ref.la
libsns_msg_motor_ref_la_SOURCES = src/msg_plugin.c
libsns_msg_motor_ref_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_motor_ref
libsns_msg_motor_ref_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_tag_motor_ref.la
libsns_msg_tag_motor_ref_la_SOURCES = src/msg_plugin.c
libsns_msg_tag_motor_ref_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_tag_motor_ref
libsns_msg_tag_motor_ref_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_motor_state.la
libsns_msg_motor_state_la_SOURCES = src/msg_plugin.c
libsns_msg_motor_state_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_motor_state
libsns_msg_motor_state_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_vector.la
libsns_msg_vector_la_SOURCES = src/msg_plugin.c
libsns_msg_vector_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_vector
libsns_msg_vector_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_tf.la
libsns_msg_tf_la_SOURCES = src/msg_plugin.c
libsns_msg_tf_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_tf
libsns_msg_tf_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_wt_tf.la
libsns_msg_wt_tf_la_SOURCES = src/msg_plugin.c
libsns_msg_wt_tf_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_wt_tf
libsns_msg_wt_tf_la_LIBADD = libsns.la

lib_LTLIBRARIES += libsns_msg_tf_dx.la
libsns_msg_tf_dx_la_SOURCES = src/msg_plugin.c
libsns_msg_tf_dx_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_tf_dx
libsns_msg_tf_dx_la_LIBADD = libsns.la


lib_LTLIBRARIES += libsns_msg_sdh_tactile.la
libsns_msg_sdh_tactile_la_SOURCES = src/msg_plugin.c
libsns_msg_sdh_tactile_la_CFLAGS = $(CFLAGS) -DSNS_MSG_PLUGIN_TYPE=sns_msg_sdh_tactile
libsns_msg_sdh_tactile_la_LIBADD = libsns.la

bin_PROGRAMS = snsdump snsplot sns-trylock


snsdump_SOURCES = src/snsdump.c
snsdump_LDADD = libsns.la -lach -lamino -llapack -lblas

snsplot_SOURCES = src/snsplot.c
snsplot_LDADD = libsns.la -lach -lamino -llapack -lblas

sns_trylock_SOURCES = src/sns-trylock.c


bin_PROGRAMS += snslogd
snslogd_SOURCES = src/snslogd.c
snslogd_LDADD = libsns.la -lach -lamino -llapack -lblas

bin_PROGRAMS += snslog
snslog_SOURCES = src/snslog.c
snslog_LDADD = libsns.la -lach -lamino -llapack -lblas

bin_PROGRAMS += snsref
snsref_SOURCES = src/snsref.c
snsref_LDADD = libsns.la -lach -lamino -llapack -lblas

bin_PROGRAMS += snsrec
snsrec_SOURCES = src/snsrec.c
snsrec_LDADD = libsns.la -lach -lamino -llapack -lblas

bin_PROGRAMS += snsreced
snsreced_SOURCES = src/snsreced.c
snsreced_LDADD = libsns.la -lach -lamino -llapack -lblas

bin_PROGRAMS += snsmplex
snsmplex_SOURCES = src/bin/mplex.c
snsmplex_LDADD = libsns.la -lach-experimental -lach -lamino -llapack -lblas

install-exec-hook:
	ln -sf $(DESTDIR)$(init_ddir)/sns$(EXEEXT) \
	       $(DESTDIR)$(bindir)/sns$(EXEEXT)

include doxygen.am
