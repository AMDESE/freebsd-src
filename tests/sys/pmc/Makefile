TESTSDIR=	${TESTSBASE}/sys/pmc

ATF_TESTS_C+=	pmc_api_test
ATF_TESTS_C+=	pmc_credexec_test
ATF_TESTS_C+=	pmc_detach_test
ATF_TESTS_C+=	pmc_exec_test
.if ${MACHINE_ARCH} == "amd64"
ATF_TESTS_C+=	pmc_exterr_test
.endif
ATF_TESTS_C+=	pmc_lifecycle_test
ATF_TESTS_C+=	pmc_log_test
ATF_TESTS_C+=	pmc_throttle_test
ATF_TESTS_C+=	pmc_wrap_test

TEST_METADATA.pmc_exterr_test+=	required_user="root"
TEST_METADATA.pmc_throttle_test+=	is_exclusive="true"

LIBADD+=	pmc
LIBADD.pmc_detach_test+=	pthread
LIBADD.pmc_throttle_test+=	pthread

CFLAGS.pmc_exterr_test.c+=	-I${SRCTOP}/tests

.include <bsd.test.mk>
