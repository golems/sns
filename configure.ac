#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.65])
AC_INIT([sns], [1.1.0], [ntd@gatech.edu])
AM_INIT_AUTOMAKE([-Wall])
AC_CONFIG_SRCDIR([include/sns.h])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CC
AC_PROG_CC_C99
AC_PROG_LIBTOOL

AC_C_RESTRICT
AC_C_INLINE
AC_C_CONST

# Checks for header files.
dnl AC_CHECK_HEADERS([arpa/inet.h fcntl.h inttypes.h stdint.h stdlib.h string.h sys/socket.h syslog.h unistd.h time.h])

# Checks for typedefs, structures, and compiler characteristics.
dnl AC_HEADER_STDBOOL
dnl AC_TYPE_INT32_T
dnl AC_TYPE_MODE_T
dnl AC_TYPE_OFF_T
dnl AC_TYPE_PID_T
dnl AC_TYPE_SIZE_T
dnl AC_TYPE_SSIZE_T
dnl AC_TYPE_UINT32_T
dnl AC_TYPE_UINT64_T
dnl AC_TYPE_UINT8_T

dnl AC_CHECK_SIZEOF(void *)
dnl AC_CHECK_SIZEOF(uint64_t)
dnl AC_CHECK_SIZEOF(unsigned long)
dnl AC_CHECK_SIZEOF(unsigned long long)

# Check 32/64 safe format specifiers
dnl AC_CHECK_DECLS([PRIuPTR,PRIu64,PRIx64])

dnl AC_CHECK_FUNCS([ftruncate isascii memmove memset munmap socket strcasecmp strchr strdup strerror strtol])
AC_SEARCH_LIBS([pthread_create],[pthread])
AC_SEARCH_LIBS([clock_gettime],[rt])
AC_SEARCH_LIBS([dlopen],[dl])


# Enable maximum warnings
m4_ifdef([AX_CFLAGS_WARN_ALL],
         [AX_CFLAGS_WARN_ALL])

dnl APPEND_FLAG( [FLAG] )
dnl Check if FLAG is supported, and if so, add it

dnl The AX_CHECK_COMPILER_FLAGS maco was renamed to AX_CHECK_COMPILE_FLAG
dnl Checking for both versions
m4_ifdef([AX_CHECK_COMPILE_FLAG],
         [AC_DEFUN([APPEND_FLAG],
                   [AX_CHECK_COMPILE_FLAG([$1], [CFLAGS="$1 $CFLAGS"])])])

m4_ifdef([AX_CHECK_COMPILER_FLAGS],
         [AC_DEFUN([APPEND_FLAG],
                   [AX_CHECK_COMPILER_FLAGS([$1], [CFLAGS="$1 $CFLAGS"])])])

dnl If we found the flag checking macro, check some flags
m4_ifdef([APPEND_FLAG],
         [APPEND_FLAG([-Wextra])
          APPEND_FLAG([-Wconversion])
          APPEND_FLAG([-Wpointer-arith])
          APPEND_FLAG([-Wfloat-equal])
          APPEND_FLAG([-Wshadow])
          APPEND_FLAG([-Wwrite-strings])
          APPEND_FLAG([-Wc++-compat])])

dnl # Doxygen
dnl m4_ifdef([DX_INIT_DOXYGEN],
dnl          [DX_HTML_FEATURE(ON)
dnl           DX_PDF_FEATURE(OFF)
dnl           DX_PS_FEATURE(OFF)
dnl           DX_INIT_DOXYGEN(ach)],
dnl          [AM_CONDITIONAL([DX_COND_doc],   false)
dnl           AM_CONDITIONAL([DX_COND_html],  false)
dnl           AM_CONDITIONAL([DX_COND_chm],   false)
dnl           AM_CONDITIONAL([DX_COND_chi],   false)
dnl           AM_CONDITIONAL([DX_COND_rtf],   false)
dnl           AM_CONDITIONAL([DX_COND_xml],   false)
dnl           AM_CONDITIONAL([DX_COND_ps],    false)
dnl           AM_CONDITIONAL([DX_COND_pdf],   false)
dnl           AM_CONDITIONAL([DX_COND_latex], false)
dnl           AM_CONDITIONAL([DX_COND_man],   false)
dnl          ])

dnl # Check for programs to make documentation
dnl AC_CHECK_PROGS([HELP2MAN], [help2man])
dnl AC_CHECK_PROGS([MAN2HTML], [man2html])
dnl AC_CHECK_PROGS([DB2HTML], [db2html])
dnl AM_CONDITIONAL([HAVE_HELP2MAN], [test x$HELP2MAN != x])
dnl AM_CONDITIONAL([HAVE_MAN2HTML], [test x$MAN2HTML != x])
dnl AM_CONDITIONAL([HAVE_DB2HTML],  [test x$DB2HTML != x])

dnl # Check if we already have docs (from tarball)
dnl AC_CHECK_FILE( [$srcdir/doc/man/ach.1],           [FOUND_MANPAGES=yes])
dnl AC_CHECK_FILE( [$srcdir/doc/man/ach.html],        [FOUND_MANHTML=yes])
dnl AC_CHECK_FILE( [$srcdir/doc/manual/ach-manual.html], [FOUND_MANUAL=yes])

dnl # If we can make the docs or already have them, then we can install them
dnl AM_CONDITIONAL([HAVE_MANPAGES], [test x$HELP2MAN != x -o x$FOUND_MANPAGES = xyes])
dnl AM_CONDITIONAL([HAVE_MANHTML],  [test '(' x$HELP2MAN != x -a x$MAN2HTML != x ')' -o x$FOUND_MANHTML = xyes])
dnl AM_CONDITIONAL([HAVE_MANUAL],   [test x$DB2HTML != x -o x$FOUND_MANUAL = xyes])

dnl # Python
dnl AC_ARG_WITH([python],
dnl         AS_HELP_STRING([--with-python], [Enable python support]))

dnl AS_IF([test "x$with_python" = "xyes"],[
dnl     AM_PATH_PYTHON( [2.6],
dnl                     [AX_PYTHON_DEVEL],
dnl                     [PYTHON_VERSION=none] )
dnl     ],
dnl     [PYTHON_VERSION=none])

dnl AM_CONDITIONAL([HAVE_PYTHON],   [test x: != x$PYTHON -a x != x$PYTHON])

AC_CONFIG_FILES([Makefile])

AC_OUTPUT

dnl AC_MSG_NOTICE([])
dnl AC_MSG_NOTICE([CONFIGURATION SUMMARY])
dnl AC_MSG_NOTICE([=====================])
dnl AC_MSG_NOTICE([PREFIX:          $prefix])
dnl AC_MSG_NOTICE([PYTHON VERSION:  $PYTHON_VERSION])
