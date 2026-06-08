# Build the hwpmc grouping/multiplex regression tests.
#
# Usage from this directory:
#	make                 # builds all three test binaries
#	make check           # userland-only test (no root needed)
#	make check-root      # in-kernel tests (need hwpmc(4) + root)
#	make clean
#
# Plain Makefile (no <bsd.prog.mk>) so it works the same in-tree and
# out-of-tree.  libpmcstat is INTERNALLIB on FreeBSD - it has no
# installed .so/.a - so test_libpmcstat_group compiles libpmcstat_group.c
# directly rather than -lpmcstat.

# Disable bmake's auto-objdir; build in-place next to the sources so
# the recipes can find pmc_group_test.c et al with relative paths and
# the resulting binaries land where `make check` expects them.
.OBJDIR: ${.CURDIR}

CC?=		cc

# Auto-detects when this directory still lives at tools/regression/pmc
# in a FreeBSD source tree; override on the command line if copied.
SRCROOT?=	${.CURDIR}/../../..

LIBPMCSTAT_SRC=	${SRCROOT}/lib/libpmcstat
LIBPMCSTAT_INC=	-I${LIBPMCSTAT_SRC}

CFLAGS+=	-O -g -Wall

# Kernel ABI tests (need hwpmc(4) loaded and root to run).
KTESTS=		pmc_group_test pmc_mux_test pmc_mux_works_test

# Userland-only test (parser unit test, no PMU access).
UTESTS=		test_libpmcstat_group

ALL=		${KTESTS} ${UTESTS}

.PHONY: all check check-root clean

all: ${ALL}

pmc_group_test: ${.CURDIR}/pmc_group_test.c
	${CC} ${CFLAGS} -o ${.TARGET} ${.ALLSRC} -lpmc

pmc_mux_test: ${.CURDIR}/pmc_mux_test.c
	${CC} ${CFLAGS} -o ${.TARGET} ${.ALLSRC} -lpmc

pmc_mux_works_test: ${.CURDIR}/pmc_mux_works_test.c
	${CC} ${CFLAGS} -o ${.TARGET} ${.ALLSRC} -lpmc

# libpmcstat is INTERNALLIB (no .so installed), so build the parser TU
# directly into the test binary.
test_libpmcstat_group: ${.CURDIR}/test_libpmcstat_group.c \
		${LIBPMCSTAT_SRC}/libpmcstat_group.c
	${CC} ${CFLAGS} ${LIBPMCSTAT_INC} -o ${.TARGET} ${.ALLSRC} -lpmc

# Userland-only check: brace-list parser, no PMU.
check: test_libpmcstat_group
	./test_libpmcstat_group

# In-kernel check: requires `kldload hwpmc` and root.
check-root: ${KTESTS}
	@echo "==> pmc_group_test"
	./pmc_group_test
	@echo "==> pmc_mux_test"
	./pmc_mux_test
	@echo "==> pmc_mux_works_test"
	./pmc_mux_works_test
	@if [ -r ${.CURDIR}/pmcstat_test.sh ]; then \
		echo "==> pmcstat(8) shell tests"; \
		sh ${.CURDIR}/pmcstat_test.sh; \
	fi

clean:
	rm -f ${ALL} *.o *.core
