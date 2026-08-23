TESTSDIR=	${TESTSBASE}/sys/pmc
BINDIR=		${TESTSDIR}

ATF_TESTS_C=	pmc_wrap_test
ATF_TESTS_C+=	pmc_detach_test
ATF_TESTS_C+=	pmc_group_test
ATF_TESTS_C+=	pmc_groupread_test
ATF_TESTS_C+=	pmc_sys_group_test
ATF_TESTS_C+=	pmc_group_gate_test
ATF_TESTS_C+=	pmc_residual_test
ATF_TESTS_C+=	pmc_fork_test
ATF_TESTS_C+=	pmc_exec_test
ATF_TESTS_C+=	pmc_sys_sampling_test

PROGS+=		pmc_exec_credential_helper

LIBADD+=	pmc
LIBADD.pmc_detach_test+=	pthread
LIBADD.pmc_fork_test+=	pthread
LIBADD.pmc_residual_test+=	pthread
LIBADD.pmc_sys_sampling_test+=	pthread

.include <bsd.test.mk>
