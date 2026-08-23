# Build the hwpmc grouping/multiplex regression tests.
#
# Usage from this directory:
#	make                 # builds all test binaries and support objects
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
CXX?=		c++

# Auto-detects when this directory still lives at tools/regression/pmc
# in a FreeBSD source tree; override on the command line if copied.
SRCROOT?=	${.CURDIR}/../../..

LIBPMC_SRC=	${SRCROOT}/lib/libpmc
LIBPMC_INC=	-I${LIBPMC_SRC} -I${SRCROOT}/sys
# Force the current log ABI without letting source sys headers shadow
# installed headers reached through the C++ standard library.
CURRENT_PMCLOG_INC=	-include ${SRCROOT}/sys/sys/pmclog.h
LIBPMCSTAT_SRC=	${SRCROOT}/lib/libpmcstat
LIBPMCSTAT_INC=	-I${LIBPMCSTAT_SRC}

CFLAGS+=	-O -g -Wall
CXXFLAGS+=	-O -g -Wall

# Kernel ABI tests (need hwpmc(4) loaded and root to run).
KTESTS=		pmc_group_test pmc_mux_test pmc_mux_works_test \
		pmc_mux_drain_test pmc_mux_rotation_test pmc_group_fuzz

# Userland-only tests (no PMU access).
UTESTS=		test_libpmcstat_group test_pmclog_group_inherit_miss \
		test_pmclog_group_inherit_miss_json \
		pmcstat_group_inherit_miss_fixture \
		pmcstat_mux_warning_shim.so

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

pmc_mux_rotation_test: ${.CURDIR}/pmc_mux_rotation_test.c
	${CC} ${CFLAGS} -o ${.TARGET} ${.ALLSRC} -lpmc

# Reproduces the multiplex switch-off drain wedge; needs -lpthread for
# the pinned real-time worker threads that starve the passive drain.
pmc_mux_drain_test: ${.CURDIR}/pmc_mux_drain_test.c
	${CC} ${CFLAGS} -o ${.TARGET} ${.ALLSRC} -lpmc -lpthread

pmc_group_fuzz: ${.CURDIR}/pmc_group_fuzz.c
	${CC} ${CFLAGS} -o ${.TARGET} ${.ALLSRC} -lpmc

# libpmcstat is INTERNALLIB (no .so installed), so build the parser TU
# directly into the test binary.
test_libpmcstat_group: ${.CURDIR}/test_libpmcstat_group.c \
		${LIBPMCSTAT_SRC}/libpmcstat_group.c
	${CC} ${CFLAGS} ${LIBPMCSTAT_INC} -o ${.TARGET} ${.ALLSRC} -lpmc

test_pmclog_group_inherit_miss: \
		${.CURDIR}/test_pmclog_group_inherit_miss.c \
		${LIBPMC_SRC}/pmclog.c
	${CC} ${CFLAGS} -Wno-pointer-sign ${LIBPMC_INC} -o ${.TARGET} ${.ALLSRC}

test_pmclog_group_inherit_miss_json: \
		${.CURDIR}/test_pmclog_group_inherit_miss_json.cc \
		${LIBPMC_SRC}/libpmc_json.cc
	${CXX} ${CXXFLAGS} -I${LIBPMC_SRC} ${CURRENT_PMCLOG_INC} \
	    -o ${.TARGET} ${.ALLSRC}

pmcstat_group_inherit_miss_fixture: \
		${.CURDIR}/pmcstat_group_inherit_miss_fixture.c
	${CC} ${CFLAGS} ${CURRENT_PMCLOG_INC} -o ${.TARGET} ${.ALLSRC}

pmcstat_mux_warning_shim.so: ${.CURDIR}/pmcstat_mux_warning_shim.c
	${CC} ${CFLAGS} -fPIC -shared ${LIBPMCSTAT_INC} \
	    -I${SRCROOT}/sys -o ${.TARGET} ${.ALLSRC}

# Userland-only checks: parsers and pmcstat warning policy.
check: ${UTESTS}
	./test_libpmcstat_group
	./test_pmclog_group_inherit_miss
	./test_pmclog_group_inherit_miss_json
	PMCSTAT=${PMCSTAT} \
	    PMCGROUP_MISS_FIXTURE=${.CURDIR}/pmcstat_group_inherit_miss_fixture \
	    sh ${.CURDIR}/pmcstat_group_inherit_miss_test.sh
	PMCSTAT=${PMCSTAT} \
	    PMCSTAT_WARNING_SHIM=${.CURDIR}/pmcstat_mux_warning_shim.so \
	    sh ${.CURDIR}/pmcstat_mux_warning_test.sh

# In-kernel check: requires `kldload hwpmc` and root.
check-root: ${KTESTS}
	@echo "==> pmc_group_test"
	./pmc_group_test
	@echo "==> pmc_mux_test"
	./pmc_mux_test
	@echo "==> pmc_mux_works_test"
	./pmc_mux_works_test
	@echo "==> pmc_mux_rotation_test"
	./pmc_mux_rotation_test
	@echo "==> pmc_mux_drain_test"
	./pmc_mux_drain_test
	@echo "==> pmc_group_fuzz"
	./pmc_group_fuzz 20000
	@if [ -r ${.CURDIR}/pmcstat_test.sh ]; then \
		echo "==> pmcstat(8) shell tests"; \
		sh ${.CURDIR}/pmcstat_test.sh; \
	fi

clean:
	rm -f ${ALL} *.o *.core
