ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include

EXTRA_DIST =

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

bin_PROGRAMS += snsref
snsref_SOURCES = src/snsref.c
snsref_LDADD = libsns.la -lach -lamino

bin_PROGRAMS += snsrec
snsrec_SOURCES = src/snsrec.c
snsrec_LDADD = libsns.la -lach -lamino

bin_PROGRAMS += snsreced
snsreced_SOURCES = src/snsreced.c
snsreced_LDADD = libsns.la -lach -lamino

bin_PROGRAMS += snsmplex
snsmplex_SOURCES = src/bin/mplex.c
snsmplex_LDADD = libsns.la -lach-experimental -lach -lamino


bin_PROGRAMS += sns-ksim
sns_ksim_CFLAGS = $(SDL2_CFLAGS) $(GL_CFLAGS)
sns_ksim_SOURCES = src/bin/sns-ksim.c
sns_ksim_LDADD = libsns.la -lach-experimental -lach -lamino -lamino-gl

install-exec-hook:
	ln -sf $(DESTDIR)$(init_ddir)/sns$(EXEEXT) \
	       $(DESTDIR)$(bindir)/sns$(EXEEXT)



doc/md/copying.md: $(top_srcdir)/COPYING $(top_srcdir)/doc/md/copying.md.header
	$(MKDIR_P) doc/md/
	cat $(top_srcdir)/doc/md/copying.md.header  \
	    $(top_srcdir)/COPYING \
	    > doc/md/copying.md


doc/md/acknowledgments.md: $(top_srcdir)/AUTHORS $(top_srcdir)/doc/md/acknowledgments.md.header
	$(MKDIR_P) doc/md/
	cat $(top_srcdir)/doc/md/acknowledgments.md.header  \
	    $(top_srcdir)/AUTHORS \
	    > doc/md/acknowledgments.md


if HAVE_DOXYGEN



doxygen: Doxyfile doc/md/copying.md doc/md/acknowledgments.md
	$(DOXYGEN)

doc/manual: doxygen

doc: doxygen
	$(MKDIR_P) doc/manual/doc

else

doc/manual:
	$(MKDIR_P) doc/manual

endif

if HAVE_HELP2MAN
dist_man_MANS =

dist_man_MANS += doc/sns.1
doc/sns.1: scripts/sns $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) --version-option=--version-man --no-info -n "SNS Management Script" $(top_srcdir)/scripts/sns -o $@

dist_man_MANS += doc/snsref.1
doc/snsref.1: $(top_builddir)/snsref $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) --version-option="-V" --no-info -n "Set motor reference" $(top_builddir)/$< -o $@

dist_man_MANS += doc/snsdump.1
doc/snsdump.1: $(top_builddir)/snsdump $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) --version-option="-V" --no-info -n "Display human-readable messages" $(top_builddir)/$< -o $@

dist_man_MANS += doc/snslogd.1
doc/snslogd.1: $(top_builddir)/snslogd $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) --version-option="-V" --no-info -n "SNS Logging Daemon" $(top_builddir)/$< -o $@

dist_man_MANS += doc/sns-ksim.1
doc/sns-ksim.1: $(top_builddir)/sns-ksim $(top_srcdir)/configure.ac
	$(MKDIR_P) doc
	$(HELP2MAN) --version-option="-V" --no-info -n "Kinematic Simulator" $(top_builddir)/$< -o $@

if HAVE_GROFF

html_mans =

html_mans += doc/manual/bin/sns.html
doc/manual/bin/sns.html: doc/sns.1
	$(MKDIR_P) doc/manual/bin
	$(GROFF) -mandoc -Thtml < $< > $@


html_mans += doc/manual/bin/snsref.html
doc/manual/bin/snsref.html: doc/snsref.1
	$(MKDIR_P) doc/manual/bin
	$(GROFF) -mandoc -Thtml < $< > $@

html_mans += doc/manual/bin/snsdump.html
doc/manual/bin/snsdump.html: doc/snsdump.1
	$(MKDIR_P) doc/manual/bin
	$(GROFF) -mandoc -Thtml < $< > $@

html_mans += doc/manual/bin/snslogd.html
doc/manual/bin/snslogd.html: doc/snslogd.1
	$(MKDIR_P) doc/manual/bin
	$(GROFF) -mandoc -Thtml < $< > $@

html_mans += doc/manual/bin/sns-ksim.html
doc/manual/bin/sns-ksim.html: doc/sns-ksim.1
	$(MKDIR_P) doc/manual/bin
	$(GROFF) -mandoc -Thtml < $< > $@

EXTRA_DIST += $(html_mans)
doxygen: $(html_mans)

endif

endif



EXTRA_DIST += doc/manual
