TESTSDIR=	${TESTSBASE}/sys/pmc
BINDIR=		${TESTSDIR}

ATF_TESTS_C+=	pmc_api_test
ATF_TESTS_C+=	pmc_attach_running_test
ATF_TESTS_C+=	pmc_credexec_test
ATF_TESTS_C+=	pmc_detach_test
ATF_TESTS_C+=	pmc_exec_test
ATF_TESTS_C+=	pmc_fork_test
ATF_TESTS_C+=	pmc_group_exec_test
ATF_TESTS_C+=	pmc_group_gate_test
ATF_TESTS_C+=	pmc_group_test
ATF_TESTS_C+=	pmc_groupread_test
.if ${MACHINE_ARCH} == "amd64"
ATF_TESTS_C+=	pmc_exterr_test
.endif
ATF_TESTS_C+=	pmc_lifecycle_test
ATF_TESTS_C+=	pmc_log_test
ATF_TESTS_C+=	pmc_residual_test
ATF_TESTS_C+=	pmc_sys_group_test
ATF_TESTS_C+=	pmc_sys_sampling_test
ATF_TESTS_C+=	pmc_wrap_test

TEST_METADATA.pmc_attach_running_test+=	required_user="root"
TEST_METADATA.pmc_exterr_test+=	required_user="root"

PROGS+=		pmc_exec_credential_helper

LIBADD+=	pmc
LIBADD.pmc_detach_test+=	pthread
LIBADD.pmc_fork_test+=	pthread
LIBADD.pmc_residual_test+=	pthread
LIBADD.pmc_sys_sampling_test+=	pthread

CFLAGS.pmc_exterr_test.c+=	-I${SRCTOP}/tests

.include <bsd.test.mk>
