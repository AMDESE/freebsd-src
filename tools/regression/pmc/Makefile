# This Makefile builds the hwpmc grouping and multiplex regression tests.
#
# Usage from this directory:
#	make                 # Build all test binaries and support objects.
#	make check           # Run the userland-only tests. Root is not needed.
#	make check-root      # Run the in-kernel tests. Need hwpmc(4) and root.
#	make clean
#
# This is a plain Makefile, not <bsd.prog.mk>. It works the same
# in-tree and out-of-tree. libpmcstat is INTERNALLIB on FreeBSD.
# It has no installed .so or .a file. test_libpmcstat_group compiles
# libpmcstat_group.c directly instead of using -lpmcstat.

# Disable bmake's auto-objdir. Build in-place next to the sources.
# This lets the recipes find pmc_group_test.c and other files by
# relative path. The binaries land where `make check` expects them.
.OBJDIR: ${.CURDIR}

CC?=		cc
CXX?=		c++

# This detects when this directory still lives at tools/regression/pmc
# in a FreeBSD source tree. Override on the command line if you copy it.
SRCROOT?=	${.CURDIR}/../../..

LIBPMC_SRC=	${SRCROOT}/lib/libpmc
LIBPMC_INC=	-I${LIBPMC_SRC} -I${SRCROOT}/sys
# Force the current log ABI.
# Do not let the sys headers hide the installed C++ standard library headers.
CURRENT_PMCLOG_INC=	-include ${SRCROOT}/sys/sys/pmclog.h
LIBPMCSTAT_SRC=	${SRCROOT}/lib/libpmcstat
LIBPMCSTAT_INC=	-I${LIBPMCSTAT_SRC}

CFLAGS+=	-O -g -Wall
CXXFLAGS+=	-O -g -Wall

# Kernel ABI tests (need hwpmc(4) loaded and root to run).
KTESTS=		pmc_group_test pmc_mux_test pmc_mux_works_test

# Userland-only tests (no PMU access).
UTESTS=

PMCSTAT?=	/usr/sbin/pmcstat

ALL=		${KTESTS} ${UTESTS}

.PHONY: all check check-root clean

all: ${ALL}

pmc_group_test: ${.CURDIR}/pmc_group_test.c
	${CC} ${CFLAGS} -o ${.TARGET} ${.ALLSRC} -lpmc

pmc_mux_test: ${.CURDIR}/pmc_mux_test.c
	${CC} ${CFLAGS} -o ${.TARGET} ${.ALLSRC} -lpmc

pmc_mux_works_test: ${.CURDIR}/pmc_mux_works_test.c
	${CC} ${CFLAGS} -o ${.TARGET} ${.ALLSRC} -lpmc

# These are userland-only checks: parsers and pmcstat warning policy.
check: ${UTESTS}

# This in-kernel check needs `kldload hwpmc` and root.
check-root: ${KTESTS}
	@echo "==> pmc_group_test"
	./pmc_group_test
	@echo "==> pmc_mux_test"
	./pmc_mux_test
	@echo "==> pmc_mux_works_test"
	./pmc_mux_works_test

clean:
	rm -f ${ALL} *.o *.core
