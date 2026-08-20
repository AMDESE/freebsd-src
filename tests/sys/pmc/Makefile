TESTSDIR=	${TESTSBASE}/sys/pmc

ATF_TESTS_C=	pmc_wrap_test
ATF_TESTS_C+=	pmc_detach_test
ATF_TESTS_C+=	pmc_group_test
ATF_TESTS_C+=	pmc_groupread_test
ATF_TESTS_C+=	pmc_sys_group_test
ATF_TESTS_C+=	pmc_group_gate_test
ATF_TESTS_C+=	pmc_residual_test

LIBADD+=	pmc
LIBADD.pmc_detach_test+=	pthread

.include <bsd.test.mk>
