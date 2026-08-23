/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

/*
 * Sampling residuals must survive eviction.
 *
 * A multiplexed group is on hardware for only part of each rotation.  If
 * progress toward the next sample were discarded at eviction, a member
 * whose period is longer than one residency window would restart its climb
 * every window and never reach an overflow -- zero samples, indefinitely,
 * however long the run.  With the residual preserved, progress accumulates
 * across windows and delivery converges on residency times the natural
 * rate.
 *
 * The test picks a period deliberately larger than what one window can
 * retire, oversubscribes the PMU so rotation is forced, and requires
 * samples to arrive anyway.  A control group with the same period and no
 * multiplexing shows the period itself is reachable.
 */

#include <sys/types.h>
#include <sys/sysctl.h>
#include <sys/thr.h>

#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <pmc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <atf-c.h>

#define	TEST_EVENT	"instructions"
#define	TEST_SYSCTL_PREFIX	"kern.hwpmc.test."
#define	TEST_HOLD_GROUP_RESIDENT					\
    TEST_SYSCTL_PREFIX "hold_group_resident"
#define	TEST_HOLD_GROUP_RESIDENT_ACK				\
    TEST_SYSCTL_PREFIX "hold_group_resident_ack"
#define	TEST_HOLD_GROUP_EVICTED					\
    TEST_SYSCTL_PREFIX "hold_group_evicted"
#define	TEST_HOLD_GROUP_EVICTED_ACK				\
    TEST_SYSCTL_PREFIX "hold_group_evicted_ack"
#define	TEST_LIVE_RESIDUAL_ENTRIES				\
    TEST_SYSCTL_PREFIX "live_residual_entries"
#define	TEST_RESIDUAL_STATE					\
    TEST_SYSCTL_PREFIX "residual_state"

#define	RESIDUAL_WORKERS	2

/*
 * The period must exceed what one residency window can retire, or progress
 * never has to carry across a window and the test proves nothing.  A 50 ms
 * window retires a few hundred million events on this class of hardware,
 * so 1e9 spans roughly two windows.  The workload is sized to deliver
 * enough samples for the counts to mean something.
 */
#define	LONG_PERIOD	(1000ULL * 1000 * 1000)
#define	SPIN_ITERS	(4000ULL * 1000 * 1000)
#define	THREAD_A_RESIDUAL	(LONG_PERIOD / 4)
#define	THREAD_B_RESIDUAL	((LONG_PERIOD / 4) * 3)

enum pmc_test_residual_operation {
	PMC_TEST_RESIDUAL_GET_SAVED = 1,
	PMC_TEST_RESIDUAL_GET_LIVE,
	PMC_TEST_RESIDUAL_SET_LIVE,
	PMC_TEST_RESIDUAL_SET_SAVED
};

enum pmc_test_residual_kind {
	PMC_TEST_RESIDUAL_UNINITIALIZED,
	PMC_TEST_RESIDUAL_VALID,
	PMC_TEST_RESIDUAL_OVERFLOW_PENDING
};

struct pmc_test_residual_query {
	uint64_t	ptrq_handle;
	uint64_t	ptrq_value;
	uint32_t	ptrq_pid;
	uint32_t	ptrq_tid;
	uint32_t	ptrq_operation;
	uint32_t	ptrq_state;
	uint32_t	ptrq_assigned;
	uint32_t	ptrq_reserved;
};

struct group {
	uint32_t	g_id;
	pmc_id_t	g_ids[PMC_GROUP_MAX_MEMBERS];
	u_int		g_n;
	bool		g_started;
};

struct group_build_failure {
	const char	*gbf_operation;
	u_int		gbf_member;
	int		gbf_errno;
};

struct residual_worker_gate {
	pthread_mutex_t	rwg_lock;
	pthread_cond_t	rwg_cond;
	bool		rwg_release;
	u_int		rwg_ready;
	int		rwg_error;
	uint32_t	rwg_tids[RESIDUAL_WORKERS];
};

struct residual_worker_arg {
	struct residual_worker_gate	*rwa_gate;
	u_int				rwa_index;
};

static bool
is_amd(void)
{
	char vendor[64];
	size_t len;

	len = sizeof(vendor);
	if (sysctlbyname("kern.hwpmc.cpuid", vendor, &len, NULL, 0) != 0)
		return (false);
	return (strstr(vendor, "AuthenticAMD") != NULL ||
	    strstr(vendor, "HygonGenuine") != NULL);
}

static void
require_hwpmc(void)
{

	if (pmc_init() != 0)
		atf_tc_skip("hwpmc(4) is not available: %s", strerror(errno));
	if (!is_amd())
		atf_tc_skip("PMC grouping is supported only on AMD CPUs");
}

static int
sysctl_read_u32(const char *name, u_int *value)
{
	size_t length;

	length = sizeof(*value);
	return (sysctlbyname(name, value, &length, NULL, 0));
}

static int
sysctl_write_u32(const char *name, u_int value)
{

	return (sysctlbyname(name, NULL, NULL, &value, sizeof(value)));
}

static int
sysctl_read_u64(const char *name, uint64_t *value)
{
	size_t length;

	length = sizeof(*value);
	return (sysctlbyname(name, value, &length, NULL, 0));
}

static void
require_hwpmc_test_support(void)
{
	uint64_t value;
	int error;

	errno = 0;
	error = sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES, &value);
	if (error == 0)
		return;
	if (errno == ENOENT)
		atf_tc_skip("requires an HWPMCDEBUG module with test support");
	atf_tc_fail("reading %s failed: errno %d (%s)",
	    TEST_LIVE_RESIDUAL_ENTRIES, errno, strerror(errno));
}

static void
require_test_sysctl_write(const char *name, u_int value)
{

	ATF_REQUIRE_MSG(sysctl_write_u32(name, value) == 0,
	    "writing %s=%u failed: errno %d (%s)", name, value, errno,
	    strerror(errno));
}

static void
wait_test_sysctl_ack(const char *name, u_int expected)
{
	u_int value;
	int i;

	for (i = 0; i < 5000; i++) {
		ATF_REQUIRE_MSG(sysctl_read_u32(name, &value) == 0,
		    "reading %s failed: errno %d (%s)", name, errno,
		    strerror(errno));
		if (value == expected)
			return;
		usleep(1000);
	}
	atf_tc_fail("%s did not acknowledge value %u", name, expected);
}

static void
reset_residual_test_hooks(void)
{

	(void)sysctl_write_u32(TEST_HOLD_GROUP_RESIDENT, 0);
	(void)sysctl_write_u32(TEST_HOLD_GROUP_EVICTED, 0);
}

static int
residual_query(pmc_id_t pmcid, uint32_t tid, uint32_t operation,
    uint32_t state, pmc_value_t value, struct pmc_test_residual_query *result)
{
	struct pmc_test_residual_query request;
	size_t length;

	memset(&request, 0, sizeof(request));
	request.ptrq_handle = pmcid;
	request.ptrq_value = value;
	request.ptrq_pid = getpid();
	request.ptrq_tid = tid;
	request.ptrq_operation = operation;
	request.ptrq_state = state;
	length = sizeof(*result);
	memset(result, 0, sizeof(*result));
	if (sysctlbyname(TEST_RESIDUAL_STATE, result, &length, &request,
	    sizeof(request)) != 0)
		return (-1);
	if (length != sizeof(*result)) {
		errno = EIO;
		return (-1);
	}
	return (0);
}

static void
set_live_residual_required(pmc_id_t pmcid, uint32_t tid, pmc_value_t value)
{
	struct pmc_test_residual_query result;

	ATF_REQUIRE_MSG(residual_query(pmcid, tid,
	    PMC_TEST_RESIDUAL_SET_LIVE, PMC_TEST_RESIDUAL_VALID, value,
	    &result) == 0,
	    "setting live residual for TID %u failed: errno %d (%s)", tid,
	    errno, strerror(errno));
	ATF_REQUIRE_MSG(result.ptrq_assigned == 1 &&
	    result.ptrq_state == PMC_TEST_RESIDUAL_VALID &&
	    result.ptrq_value == value,
	    "live residual injection for TID %u returned assigned=%u "
	    "state=%u value=%ju", tid, result.ptrq_assigned,
	    result.ptrq_state, (uintmax_t)result.ptrq_value);
}

static void
get_residual_required(pmc_id_t pmcid, uint32_t tid, uint32_t operation,
    struct pmc_test_residual_query *result)
{

	ATF_REQUIRE_MSG(residual_query(pmcid, tid, operation,
	    PMC_TEST_RESIDUAL_UNINITIALIZED, 0, result) == 0,
	    "reading residual state for TID %u operation %u failed: "
	    "errno %d (%s)", tid, operation, errno, strerror(errno));
}

static void *
residual_worker(void *arg) __no_lock_analysis
{
	struct residual_worker_arg *worker;
	struct residual_worker_gate *gate;
	long tid;
	int error, wait_error;

	worker = arg;
	gate = worker->rwa_gate;
	errno = 0;
	error = thr_self(&tid) == 0 ? 0 : errno;
	if (error == 0 && (tid <= 0 || (uint64_t)tid > UINT32_MAX))
		error = EOVERFLOW;

	wait_error = pthread_mutex_lock(&gate->rwg_lock);
	if (wait_error != 0)
		return ((void *)(uintptr_t)wait_error);
	if (error == 0)
		gate->rwg_tids[worker->rwa_index] = (uint32_t)tid;
	else if (gate->rwg_error == 0)
		gate->rwg_error = error;
	gate->rwg_ready++;
	(void)pthread_cond_broadcast(&gate->rwg_cond);
	while (!gate->rwg_release && gate->rwg_error == 0) {
		wait_error = pthread_cond_wait(&gate->rwg_cond, &gate->rwg_lock);
		if (wait_error != 0) {
			gate->rwg_error = wait_error;
			(void)pthread_cond_broadcast(&gate->rwg_cond);
			break;
		}
	}
	error = gate->rwg_error;
	(void)pthread_mutex_unlock(&gate->rwg_lock);
	return ((void *)(uintptr_t)error);
}

static void
residual_workers_start(struct residual_worker_gate *gate,
    pthread_t threads[RESIDUAL_WORKERS],
    struct residual_worker_arg args[RESIDUAL_WORKERS], u_int nworkers)
{
	struct timespec deadline;
	u_int i;
	int error;

	ATF_REQUIRE_MSG(nworkers > 0 && nworkers <= RESIDUAL_WORKERS,
	    "invalid residual worker count %u", nworkers);
	memset(gate, 0, sizeof(*gate));
	error = pthread_mutex_init(&gate->rwg_lock, NULL);
	ATF_REQUIRE_MSG(error == 0, "pthread_mutex_init failed: %d (%s)",
	    error, strerror(error));
	error = pthread_cond_init(&gate->rwg_cond, NULL);
	ATF_REQUIRE_MSG(error == 0, "pthread_cond_init failed: %d (%s)",
	    error, strerror(error));

	for (i = 0; i < nworkers; i++) {
		args[i].rwa_gate = gate;
		args[i].rwa_index = i;
		error = pthread_create(&threads[i], NULL, residual_worker,
		    &args[i]);
		ATF_REQUIRE_MSG(error == 0,
		    "pthread_create for residual worker %u failed: %d (%s)",
		    i, error, strerror(error));
	}

	ATF_REQUIRE_MSG(clock_gettime(CLOCK_REALTIME, &deadline) == 0,
	    "clock_gettime failed: %s", strerror(errno));
	deadline.tv_sec += 5;
	error = pthread_mutex_lock(&gate->rwg_lock);
	ATF_REQUIRE_MSG(error == 0, "pthread_mutex_lock failed: %d (%s)",
	    error, strerror(error));
	while (gate->rwg_ready != nworkers && gate->rwg_error == 0) {
		error = pthread_cond_timedwait(&gate->rwg_cond, &gate->rwg_lock,
		    &deadline);
		if (error != 0)
			break;
	}
	ATF_REQUIRE_MSG(pthread_mutex_unlock(&gate->rwg_lock) == 0,
	    "pthread_mutex_unlock failed");
	ATF_REQUIRE_MSG(error == 0,
	    "waiting for residual workers failed: %d (%s)", error,
	    strerror(error));
	ATF_REQUIRE_MSG(gate->rwg_error == 0,
	    "residual worker setup failed: %d (%s)", gate->rwg_error,
	    strerror(gate->rwg_error));
	ATF_REQUIRE_EQ(gate->rwg_ready, nworkers);
}

static void
residual_workers_stop(struct residual_worker_gate *gate,
    pthread_t threads[RESIDUAL_WORKERS], u_int nworkers)
{
	void *result;
	u_int i;
	int error;

	ATF_REQUIRE_MSG(nworkers > 0 && nworkers <= RESIDUAL_WORKERS,
	    "invalid residual worker count %u", nworkers);
	error = pthread_mutex_lock(&gate->rwg_lock);
	ATF_REQUIRE_MSG(error == 0, "pthread_mutex_lock failed: %d (%s)",
	    error, strerror(error));
	gate->rwg_release = true;
	error = pthread_cond_broadcast(&gate->rwg_cond);
	ATF_REQUIRE_MSG(error == 0, "pthread_cond_broadcast failed: %d (%s)",
	    error, strerror(error));
	error = pthread_mutex_unlock(&gate->rwg_lock);
	ATF_REQUIRE_MSG(error == 0, "pthread_mutex_unlock failed: %d (%s)",
	    error, strerror(error));

	for (i = 0; i < nworkers; i++) {
		error = pthread_join(threads[i], &result);
		ATF_REQUIRE_MSG(error == 0,
		    "pthread_join for residual worker %u failed: %d (%s)",
		    i, error, strerror(error));
		ATF_REQUIRE_MSG(result == NULL,
		    "residual worker %u returned error %ju", i,
		    (uintmax_t)(uintptr_t)result);
	}
	error = pthread_cond_destroy(&gate->rwg_cond);
	ATF_REQUIRE_MSG(error == 0, "pthread_cond_destroy failed: %d (%s)",
	    error, strerror(error));
	error = pthread_mutex_destroy(&gate->rwg_lock);
	ATF_REQUIRE_MSG(error == 0, "pthread_mutex_destroy failed: %d (%s)",
	    error, strerror(error));
}

static void
group_init(struct group *g)
{
	u_int i;

	memset(g, 0, sizeof(*g));
	for (i = 0; i < nitems(g->g_ids); i++)
		g->g_ids[i] = PMC_ID_INVALID;
}

static void
group_teardown(struct group *g)
{
	u_int i;

	if (g->g_started)
		(void)pmc_stop(g->g_ids[0]);
	for (i = 0; i < g->g_n; i++) {
		if (g->g_ids[i] != PMC_ID_INVALID)
			(void)pmc_release(g->g_ids[i]);
	}
	group_init(g);
}

static void
group_teardown_required(struct group *g, const char *description)
{

	if (g->g_started) {
		ATF_REQUIRE_MSG(pmc_stop(g->g_ids[0]) == 0,
		    "%s stop failed: errno %d (%s)", description, errno,
		    strerror(errno));
		g->g_started = false;
	}
	if (g->g_n != 0 && g->g_ids[0] != PMC_ID_INVALID) {
		ATF_REQUIRE_MSG(pmc_release(g->g_ids[0]) == 0,
		    "%s leader release failed: errno %d (%s)", description,
		    errno, strerror(errno));
	}
	group_init(g);
}

/*
 * Build and commit an n-member sampling group whose leader is member 0.
 * With mux set, the group may be time-multiplexed against others.
 */
static int
group_build(struct group *g, u_int n, bool mux,
    struct group_build_failure *failure)
{
	uint32_t flags;
	u_int i;

	failure->gbf_operation = NULL;
	failure->gbf_member = UINT_MAX;
	failure->gbf_errno = 0;
	group_init(g);
	g->g_n = n;
	for (i = 0; i < n; i++) {
		flags = (i == 0 && mux) ? PMC_F_GROUP_MUX : 0;
		if (pmc_allocate_group(TEST_EVENT, PMC_MODE_TS, flags,
		    PMC_CPU_ANY, &g->g_ids[i], LONG_PERIOD) != 0) {
			failure->gbf_operation = "pmc_allocate_group";
			failure->gbf_member = i;
			failure->gbf_errno = errno;
			return (-1);
		}
	}
	if (pmc_group_create(&g->g_id) != 0) {
		failure->gbf_operation = "pmc_group_create";
		failure->gbf_errno = errno;
		return (-1);
	}
	for (i = 0; i < n; i++) {
		if (pmc_group_add(g->g_id, g->g_ids[i], i == 0) != 0) {
			failure->gbf_operation = "pmc_group_add";
			failure->gbf_member = i;
			failure->gbf_errno = errno;
			return (-1);
		}
	}
	if (pmc_group_commit(g->g_id) != 0) {
		failure->gbf_operation = "pmc_group_commit";
		failure->gbf_errno = errno;
		return (-1);
	}
	return (0);
}

static void
format_group_build_failure(char *buffer, size_t size, const char *description,
    const struct group_build_failure *failure)
{
	const char *operation;

	operation = failure->gbf_operation != NULL ?
	    failure->gbf_operation : "unknown operation";
	if (failure->gbf_member == UINT_MAX) {
		snprintf(buffer, size, "%s failed at %s: errno %d (%s)",
		    description, operation, failure->gbf_errno,
		    strerror(failure->gbf_errno));
	} else {
		snprintf(buffer, size,
		    "%s failed at %s for member %u: errno %d (%s)",
		    description, operation, failure->gbf_member,
		    failure->gbf_errno, strerror(failure->gbf_errno));
	}
}

static void
group_build_required(struct group *g, u_int n, bool mux,
    const char *description)
{
	struct group_build_failure failure;
	char message[256];

	ATF_REQUIRE_MSG(n > 0 && n <= PMC_GROUP_MAX_MEMBERS,
	    "%s requested invalid member count %u", description, n);
	if (group_build(g, n, mux, &failure) == 0)
		return;
	format_group_build_failure(message, sizeof(message), description,
	    &failure);
	group_teardown(g);
	atf_tc_fail("%s", message);
}

/* Largest group that commits, which is the usable row capacity. */
static u_int
probe_capacity(void)
{
	struct group_build_failure failure;
	struct group probe;
	char description[128], message[256];
	u_int n;

	for (n = PMC_GROUP_MAX_MEMBERS; n > 0; n--) {
		if (group_build(&probe, n, false, &failure) == 0) {
			group_teardown(&probe);
			return (n);
		}
		group_teardown(&probe);
		snprintf(description, sizeof(description),
		    "capacity probe for %u members", n);
		if (failure.gbf_operation == NULL ||
		    strcmp(failure.gbf_operation, "pmc_group_commit") != 0 ||
		    failure.gbf_errno != ENOSPC) {
			format_group_build_failure(message, sizeof(message),
			    description, &failure);
			atf_tc_fail("%s; expected pmc_group_commit to fail with "
			    "ENOSPC while reducing the probe size", message);
		}
	}
	return (0);
}

static void __attribute__((noinline))
spin(void)
{
	volatile uint64_t sink;
	uint64_t i;

	sink = 0;
	for (i = 0; i < SPIN_ITERS; i++)
		sink += i;
}

/*
 * Delivered sample count of the leader, which pm_value reports, and the
 * group's residency.  The buffer must hold every member: a short one fails
 * E2BIG and reads nothing.
 */
static pmc_value_t
leader_samples(struct group *g, double *residency)
{
	struct pmc_group_member m[PMC_GROUP_MAX_MEMBERS];
	struct pmc_group_times t;
	uint32_t n;

	n = g->g_n;
	memset(m, 0, sizeof(m));
	memset(&t, 0, sizeof(t));
	ATF_REQUIRE_MSG(pmc_group_read(g->g_ids[0], &n, m, &t) == 0,
	    "group read failed: %s", strerror(errno));
	ATF_REQUIRE_EQ(n, g->g_n);
	if (residency != NULL) {
		*residency = t.pgt_enabled != 0 ?
		    (double)t.pgt_running / t.pgt_enabled : 0.0;
	}
	return (m[0].pm_value);
}

static void
run_group(struct group *g)
{

	ATF_REQUIRE_MSG(pmc_attach(g->g_ids[0], getpid()) == 0,
	    "attach failed: %s", strerror(errno));
	ATF_REQUIRE_MSG(pmc_start(g->g_ids[0]) == 0, "start failed: %s",
	    strerror(errno));
	g->g_started = true;
}

/*
 * Control: a period this long is reachable when the group is never evicted.
 * If this fails the multiplexed case below proves nothing.
 */
ATF_TC_WITHOUT_HEAD(long_period_unmultiplexed);
ATF_TC_BODY(long_period_unmultiplexed, tc)
{
	struct group g;
	FILE *log;
	pmc_value_t samples;

	require_hwpmc();
	log = tmpfile();
	ATF_REQUIRE(log != NULL);
	ATF_REQUIRE(pmc_configure_logfile(fileno(log)) == 0);

	group_build_required(&g, 1, false, "unmultiplexed group");
	run_group(&g);
	spin();
	samples = leader_samples(&g, NULL);
	group_teardown(&g);
	(void)pmc_configure_logfile(-1);
	(void)fclose(log);

	printf("unmultiplexed: %ju samples\n", (uintmax_t)samples);
	ATF_CHECK_MSG(samples > 0,
	    "no samples without multiplexing: the period is unreachable in "
	    "this workload, so the multiplexed case cannot be judged");
}

/*
 * The real check.  Two oversubscribed groups force rotation, so neither is
 * resident continuously.  Without residual preservation the leader climbs
 * from zero every window and never overflows.
 */
ATF_TC_WITHOUT_HEAD(long_period_under_mux);
ATF_TC_BODY(long_period_under_mux, tc)
{
	struct group_build_failure failure;
	struct group a, b;
	char message[256];
	FILE *log;
	pmc_value_t samples;
	double residency;
	u_int cap;

	require_hwpmc();
	cap = probe_capacity();
	if (cap < 2)
		atf_tc_skip("need at least two group rows, have %u", cap);

	log = tmpfile();
	ATF_REQUIRE(log != NULL);
	ATF_REQUIRE(pmc_configure_logfile(fileno(log)) == 0);

	group_build_required(&a, cap, true, "first multiplexed group");
	if (group_build(&b, cap, true, &failure) != 0) {
		format_group_build_failure(message, sizeof(message),
		    "second multiplexed group", &failure);
		group_teardown(&b);
		group_teardown(&a);
		(void)pmc_configure_logfile(-1);
		(void)fclose(log);
		atf_tc_fail("%s", message);
	}
	run_group(&a);
	run_group(&b);
	spin();
	samples = leader_samples(&a, &residency);
	group_teardown(&b);
	group_teardown(&a);
	(void)pmc_configure_logfile(-1);
	(void)fclose(log);

	printf("multiplexed: %ju samples at %.1f%% residency\n",
	    (uintmax_t)samples, residency * 100.0);
	ATF_CHECK_MSG(residency < 0.95,
	    "the group was resident %.1f%% of the time, so it was never "
	    "really multiplexed and the check below proves nothing",
	    residency * 100.0);
	ATF_CHECK_MSG(samples > 0,
	    "a multiplexed member whose period exceeds one residency window "
	    "delivered no samples: the residual was discarded at eviction");
}

/*
 * Saved residual state must retain identity independently of its numeric
 * value.  In particular, an overflow pending at schedule-out and an
 * uninitialized event both carry value zero but are not the same state.
 */
ATF_TC_WITH_CLEANUP(residual_state_identity);
ATF_TC_HEAD(residual_state_identity, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "15");
}
ATF_TC_BODY(residual_state_identity, tc)
{
	static const struct {
		uint32_t	state;
		pmc_value_t	value;
		const char	*name;
	} cases[] = {
		{ PMC_TEST_RESIDUAL_UNINITIALIZED, 0, "uninitialized" },
		{ PMC_TEST_RESIDUAL_VALID, THREAD_A_RESIDUAL, "valid" },
		{ PMC_TEST_RESIDUAL_OVERFLOW_PENDING, 0, "overflow pending" },
	};
	struct pmc_test_residual_query observed[nitems(cases)];
	struct pmc_test_residual_query result;
	struct group g;
	uint64_t residual_entries_after, residual_entries_before;
	char failure[256];
	long tid;
	u_int i;
	int error, query_errno;

	memset(observed, 0, sizeof(observed));
	failure[0] = '\0';
	require_hwpmc();
	require_hwpmc_test_support();
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &residual_entries_before) == 0,
	    "reading initial residual-entry count failed: errno %d (%s)",
	    errno, strerror(errno));

	group_build_required(&g, 1, false, "residual-state group");
	if (pmc_attach(g.g_ids[0], getpid()) != 0) {
		query_errno = errno;
		group_teardown(&g);
		atf_tc_fail("residual-state group attach failed: errno %d (%s)",
		    query_errno, strerror(query_errno));
	}
	tid = 0;
	errno = 0;
	error = thr_self(&tid) == 0 ? 0 : errno;
	if (error != 0 || tid <= 0 || (uint64_t)tid > UINT32_MAX) {
		group_teardown(&g);
		atf_tc_fail("thr_self failed or returned invalid TID %ld: "
		    "errno %d (%s)", tid, error, strerror(error));
	}

	for (i = 0; i < nitems(cases); i++) {
		errno = 0;
		if (residual_query(g.g_ids[0], (uint32_t)tid,
		    PMC_TEST_RESIDUAL_SET_SAVED, cases[i].state,
		    cases[i].value, &result) != 0) {
			query_errno = errno;
			snprintf(failure, sizeof(failure),
			    "setting %s saved residual failed: errno %d (%s)",
			    cases[i].name, query_errno, strerror(query_errno));
			break;
		}
		errno = 0;
		if (residual_query(g.g_ids[0], (uint32_t)tid,
		    PMC_TEST_RESIDUAL_GET_SAVED,
		    PMC_TEST_RESIDUAL_UNINITIALIZED, 0,
		    &observed[i]) != 0) {
			query_errno = errno;
			snprintf(failure, sizeof(failure),
			    "reading %s saved residual failed: errno %d (%s)",
			    cases[i].name, query_errno, strerror(query_errno));
			break;
		}
		/*
		 * ptrq_assigned reports the group's current row assignment,
		 * not whether saved residual state exists.
		 */
	}

	group_teardown_required(&g, "residual-state group");
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &residual_entries_after) == 0,
	    "reading final residual-entry count failed: errno %d (%s)",
	    errno, strerror(errno));
	ATF_REQUIRE_MSG(residual_entries_after == residual_entries_before,
	    "residual entries changed from %ju to %ju after cleanup",
	    (uintmax_t)residual_entries_before,
	    (uintmax_t)residual_entries_after);
	ATF_REQUIRE_MSG(failure[0] == '\0', "%s", failure);

	for (i = 0; i < nitems(cases); i++) {
		ATF_CHECK_MSG(observed[i].ptrq_state == cases[i].state &&
		    observed[i].ptrq_value == cases[i].value,
		    "%s saved residual expected state=%u value=%ju, "
		    "observed state=%u value=%ju", cases[i].name,
		    cases[i].state, (uintmax_t)cases[i].value,
		    observed[i].ptrq_state,
		    (uintmax_t)observed[i].ptrq_value);
	}
}
ATF_TC_CLEANUP(residual_state_identity, tc)
{

	reset_residual_test_hooks();
}

/*
 * A thread created while a sampling event is evicted has no saved progress for
 * that event.  It must start with a full period when the event is placed again,
 * rather than inheriting an older thread's residual.
 */
ATF_TC_WITH_CLEANUP(thread_residual_new_thread_starts_full_period);
ATF_TC_HEAD(thread_residual_new_thread_starts_full_period, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(thread_residual_new_thread_starts_full_period, tc)
{
	struct pmc_test_residual_query new_thread_live, saved;
	struct residual_worker_arg old_args[RESIDUAL_WORKERS];
	struct residual_worker_arg new_args[RESIDUAL_WORKERS];
	struct residual_worker_gate old_gate, new_gate;
	pthread_t old_threads[RESIDUAL_WORKERS];
	pthread_t new_threads[RESIDUAL_WORKERS];
	struct group a, b;
	uint64_t residual_entries_after, residual_entries_before;
	FILE *log;
	u_int cap;

	require_hwpmc();
	require_hwpmc_test_support();
	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, 0);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, 0);
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &residual_entries_before) == 0,
	    "reading initial residual-entry count failed: errno %d (%s)",
	    errno, strerror(errno));

	/*
	 * The "old" thread exists before eviction and carries a distinct
	 * residual.  It is parked in pthread_cond_wait so its injected
	 * per-thread value is not overwritten by a normal context-switch-out
	 * before eviction snapshots it.
	 */
	residual_workers_start(&old_gate, old_threads, old_args, 1);

	log = tmpfile();
	ATF_REQUIRE_MSG(log != NULL, "tmpfile failed: errno %d (%s)", errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_configure_logfile(fileno(log)) == 0,
	    "logfile configuration failed: errno %d (%s)", errno,
	    strerror(errno));

	cap = probe_capacity();
	ATF_REQUIRE_MSG(cap > 0, "no usable %s group rows were found",
	    TEST_EVENT);
	group_build_required(&a, cap, true, "new-thread residual group");
	group_build_required(&b, cap, true, "new-thread competitor group");
	ATF_REQUIRE_MSG(pmc_attach(a.g_ids[0], getpid()) == 0,
	    "new-thread residual-group attach failed: errno %d (%s)", errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_attach(b.g_ids[0], getpid()) == 0,
	    "new-thread competitor-group attach failed: errno %d (%s)", errno,
	    strerror(errno));

	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, a.g_id);
	ATF_REQUIRE_MSG(pmc_start(a.g_ids[0]) == 0,
	    "new-thread residual-group start failed: errno %d (%s)", errno,
	    strerror(errno));
	a.g_started = true;
	wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK, a.g_id);
	ATF_REQUIRE_MSG(pmc_start(b.g_ids[0]) == 0,
	    "new-thread competitor-group start failed: errno %d (%s)", errno,
	    strerror(errno));
	b.g_started = true;

	set_live_residual_required(a.g_ids[0], old_gate.rwg_tids[0],
	    THREAD_A_RESIDUAL);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, a.g_id);
	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, 0);
	wait_test_sysctl_ack(TEST_HOLD_GROUP_EVICTED_ACK, a.g_id);
	get_residual_required(a.g_ids[0], old_gate.rwg_tids[0],
	    PMC_TEST_RESIDUAL_GET_SAVED, &saved);

	/*
	 * The "new" thread is created while the event is evicted, so it has no
	 * saved progress for it.  When the group is placed again it must start
	 * from a full period rather than inherit the old thread's residual.
	 */
	residual_workers_start(&new_gate, new_threads, new_args, 1);
	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, a.g_id);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, 0);
	wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK, a.g_id);
	get_residual_required(a.g_ids[0], new_gate.rwg_tids[0],
	    PMC_TEST_RESIDUAL_GET_LIVE, &new_thread_live);

	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, 0);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, 0);
	group_teardown_required(&b, "new-thread competitor group");
	group_teardown_required(&a, "new-thread residual group");
	ATF_REQUIRE_MSG(pmc_configure_logfile(-1) == 0,
	    "logfile close failed: errno %d (%s)", errno, strerror(errno));
	ATF_REQUIRE_MSG(fclose(log) == 0,
	    "temporary logfile close failed: errno %d (%s)", errno,
	    strerror(errno));
	residual_workers_stop(&new_gate, new_threads, 1);
	residual_workers_stop(&old_gate, old_threads, 1);
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &residual_entries_after) == 0,
	    "reading final residual-entry count failed: errno %d (%s)",
	    errno, strerror(errno));

	ATF_CHECK_MSG(residual_entries_after == residual_entries_before,
	    "residual entries changed from %ju to %ju after cleanup",
	    (uintmax_t)residual_entries_before,
	    (uintmax_t)residual_entries_after);
	ATF_CHECK_MSG(saved.ptrq_assigned == 0 &&
	    saved.ptrq_state == PMC_TEST_RESIDUAL_VALID &&
	    saved.ptrq_value == THREAD_A_RESIDUAL,
	    "existing thread expected evicted saved VALID/%ju, observed "
	    "assigned=%u state=%u value=%ju", (uintmax_t)THREAD_A_RESIDUAL,
	    saved.ptrq_assigned, saved.ptrq_state,
	    (uintmax_t)saved.ptrq_value);
	ATF_CHECK_MSG(new_thread_live.ptrq_assigned == 1 &&
	    new_thread_live.ptrq_state == PMC_TEST_RESIDUAL_VALID &&
	    new_thread_live.ptrq_value == LONG_PERIOD,
	    "new thread expected full-period VALID/%ju, observed assigned=%u "
	    "state=%u value=%ju", (uintmax_t)LONG_PERIOD,
	    new_thread_live.ptrq_assigned, new_thread_live.ptrq_state,
	    (uintmax_t)new_thread_live.ptrq_value);
}
ATF_TC_CLEANUP(thread_residual_new_thread_starts_full_period, tc)
{

	reset_residual_test_hooks();
}

/*
 * A saved residual belongs to one stable event/TID pair.  When that thread
 * exits, its saved entry must be reclaimed even while the group remains
 * attached to the process.
 */
ATF_TC_WITH_CLEANUP(thread_residual_thread_exit_reclaims_state);
ATF_TC_HEAD(thread_residual_thread_exit_reclaims_state, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "15");
}
ATF_TC_BODY(thread_residual_thread_exit_reclaims_state, tc)
{
	struct pmc_test_residual_query saved;
	struct residual_worker_arg args[RESIDUAL_WORKERS];
	struct residual_worker_gate gate;
	pthread_t threads[RESIDUAL_WORKERS];
	struct group g;
	uint64_t entries_after_cleanup, entries_after_exit;
	uint64_t entries_before, entries_while_saved;

	require_hwpmc();
	require_hwpmc_test_support();
	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, 0);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, 0);
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &entries_before) == 0,
	    "reading initial residual-entry count failed: errno %d (%s)",
	    errno, strerror(errno));
	ATF_REQUIRE_MSG(entries_before != UINT64_MAX,
	    "initial residual-entry count cannot be incremented");

	residual_workers_start(&gate, threads, args, 1);
	group_build_required(&g, 1, false, "thread-exit residual group");
	ATF_REQUIRE_MSG(pmc_attach(g.g_ids[0], getpid()) == 0,
	    "thread-exit residual-group attach failed: errno %d (%s)", errno,
	    strerror(errno));

	ATF_REQUIRE_MSG(residual_query(g.g_ids[0], gate.rwg_tids[0],
	    PMC_TEST_RESIDUAL_SET_SAVED, PMC_TEST_RESIDUAL_VALID,
	    THREAD_A_RESIDUAL, &saved) == 0,
	    "setting saved residual for exiting TID %u failed: errno %d (%s)",
	    gate.rwg_tids[0], errno, strerror(errno));
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &entries_while_saved) == 0,
	    "reading live residual-entry count failed: errno %d (%s)",
	    errno, strerror(errno));

	residual_workers_stop(&gate, threads, 1);
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &entries_after_exit) == 0,
	    "reading post-exit residual-entry count failed: errno %d (%s)",
	    errno, strerror(errno));

	group_teardown_required(&g, "thread-exit residual group");
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &entries_after_cleanup) == 0,
	    "reading final residual-entry count failed: errno %d (%s)",
	    errno, strerror(errno));

	ATF_CHECK_MSG(saved.ptrq_state == PMC_TEST_RESIDUAL_VALID &&
	    saved.ptrq_value == THREAD_A_RESIDUAL,
	    "exiting thread saved residual expected VALID/%ju, observed "
	    "group_assigned=%u state=%u value=%ju",
	    (uintmax_t)THREAD_A_RESIDUAL,
	    saved.ptrq_assigned, saved.ptrq_state,
	    (uintmax_t)saved.ptrq_value);
	ATF_CHECK_MSG(entries_while_saved == entries_before + 1,
	    "saved residual entry count was %ju, expected initial %ju + 1",
	    (uintmax_t)entries_while_saved, (uintmax_t)entries_before);
	ATF_CHECK_MSG(entries_after_exit == entries_before,
	    "residual entries after thread exit were %ju, expected %ju",
	    (uintmax_t)entries_after_exit, (uintmax_t)entries_before);
	ATF_CHECK_MSG(entries_after_cleanup == entries_before,
	    "residual entries after cleanup were %ju, expected %ju",
	    (uintmax_t)entries_after_cleanup, (uintmax_t)entries_before);
}
ATF_TC_CLEANUP(thread_residual_thread_exit_reclaims_state, tc)
{

	reset_residual_test_hooks();
}

/*
 * A sampling event has independent progress for each target thread.  Inject
 * two distinct live residuals, force a real eviction and re-entry, and require
 * each TID to retain its own value.  The current event-wide minimum model
 * returns and restores one common value, so at least one exact assertion must
 * fail for both the saved and restored snapshots.
 */
ATF_TC_WITH_CLEANUP(per_thread_residuals_survive_eviction);
ATF_TC_HEAD(per_thread_residuals_survive_eviction, tc)
{

	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(per_thread_residuals_survive_eviction, tc)
{
	struct pmc_test_residual_query live[RESIDUAL_WORKERS];
	struct pmc_test_residual_query saved[RESIDUAL_WORKERS];
	struct residual_worker_arg args[RESIDUAL_WORKERS];
	struct residual_worker_gate gate;
	pthread_t threads[RESIDUAL_WORKERS];
	const pmc_value_t expected[RESIDUAL_WORKERS] = {
		THREAD_A_RESIDUAL,
		THREAD_B_RESIDUAL,
	};
	struct group a, b;
	uint64_t residual_entries_after, residual_entries_before;
	FILE *log;
	u_int cap, i;

	require_hwpmc();
	require_hwpmc_test_support();
	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, 0);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, 0);
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &residual_entries_before) == 0,
	    "reading initial residual-entry count failed: errno %d (%s)",
	    errno, strerror(errno));

	residual_workers_start(&gate, threads, args, RESIDUAL_WORKERS);

	log = tmpfile();
	ATF_REQUIRE_MSG(log != NULL, "tmpfile failed: errno %d (%s)", errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_configure_logfile(fileno(log)) == 0,
	    "logfile configuration failed: errno %d (%s)", errno,
	    strerror(errno));

	cap = probe_capacity();
	ATF_REQUIRE_MSG(cap > 0, "no usable %s group rows were found",
	    TEST_EVENT);
	group_build_required(&a, cap, true, "residual group");
	group_build_required(&b, cap, true, "competitor group");

	ATF_REQUIRE_MSG(pmc_attach(a.g_ids[0], getpid()) == 0,
	    "residual-group attach failed: errno %d (%s)", errno,
	    strerror(errno));
	ATF_REQUIRE_MSG(pmc_attach(b.g_ids[0], getpid()) == 0,
	    "competitor-group attach failed: errno %d (%s)", errno,
	    strerror(errno));

	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, a.g_id);
	ATF_REQUIRE_MSG(pmc_start(a.g_ids[0]) == 0,
	    "residual-group start failed: errno %d (%s)", errno,
	    strerror(errno));
	a.g_started = true;
	wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK, a.g_id);

	ATF_REQUIRE_MSG(pmc_start(b.g_ids[0]) == 0,
	    "competitor-group start failed: errno %d (%s)", errno,
	    strerror(errno));
	b.g_started = true;

	for (i = 0; i < RESIDUAL_WORKERS; i++)
		set_live_residual_required(a.g_ids[0], gate.rwg_tids[i],
		    expected[i]);

	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, a.g_id);
	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, 0);
	wait_test_sysctl_ack(TEST_HOLD_GROUP_EVICTED_ACK, a.g_id);

	for (i = 0; i < RESIDUAL_WORKERS; i++) {
		get_residual_required(a.g_ids[0], gate.rwg_tids[i],
		    PMC_TEST_RESIDUAL_GET_SAVED, &saved[i]);
		ATF_REQUIRE_MSG(saved[i].ptrq_assigned == 0,
		    "saved residual query for TID %u observed assigned=%u",
		    gate.rwg_tids[i], saved[i].ptrq_assigned);
		ATF_REQUIRE_MSG(
		    saved[i].ptrq_state == PMC_TEST_RESIDUAL_VALID,
		    "saved residual query for TID %u returned state=%u value=%ju",
		    gate.rwg_tids[i], saved[i].ptrq_state,
		    (uintmax_t)saved[i].ptrq_value);
	}

	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, a.g_id);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, 0);
	wait_test_sysctl_ack(TEST_HOLD_GROUP_RESIDENT_ACK, a.g_id);

	for (i = 0; i < RESIDUAL_WORKERS; i++) {
		get_residual_required(a.g_ids[0], gate.rwg_tids[i],
		    PMC_TEST_RESIDUAL_GET_LIVE, &live[i]);
		ATF_REQUIRE_MSG(live[i].ptrq_assigned == 1,
		    "live residual query for TID %u observed assigned=%u",
		    gate.rwg_tids[i], live[i].ptrq_assigned);
		ATF_REQUIRE_MSG(live[i].ptrq_state == PMC_TEST_RESIDUAL_VALID,
		    "live residual query for TID %u returned state=%u value=%ju",
		    gate.rwg_tids[i], live[i].ptrq_state,
		    (uintmax_t)live[i].ptrq_value);
	}

	require_test_sysctl_write(TEST_HOLD_GROUP_RESIDENT, 0);
	require_test_sysctl_write(TEST_HOLD_GROUP_EVICTED, 0);
	group_teardown_required(&b, "competitor group");
	group_teardown_required(&a, "residual group");
	ATF_REQUIRE_MSG(pmc_configure_logfile(-1) == 0,
	    "logfile close failed: errno %d (%s)", errno, strerror(errno));
	ATF_REQUIRE_MSG(fclose(log) == 0,
	    "temporary logfile close failed: errno %d (%s)", errno,
	    strerror(errno));
	residual_workers_stop(&gate, threads, RESIDUAL_WORKERS);
	ATF_REQUIRE_MSG(sysctl_read_u64(TEST_LIVE_RESIDUAL_ENTRIES,
	    &residual_entries_after) == 0,
	    "reading final residual-entry count failed: errno %d (%s)",
	    errno, strerror(errno));

	ATF_CHECK_MSG(residual_entries_after == residual_entries_before,
	    "residual entries changed from %ju to %ju after cleanup",
	    (uintmax_t)residual_entries_before,
	    (uintmax_t)residual_entries_after);
	for (i = 0; i < RESIDUAL_WORKERS; i++) {
		printf("TID %u expected=%ju saved=%ju live=%ju\n",
		    gate.rwg_tids[i], (uintmax_t)expected[i],
		    (uintmax_t)saved[i].ptrq_value,
		    (uintmax_t)live[i].ptrq_value);
		ATF_CHECK_MSG(saved[i].ptrq_value == expected[i],
		    "thread %c expected saved VALID/%ju, observed VALID/%ju",
		    'A' + i, (uintmax_t)expected[i],
		    (uintmax_t)saved[i].ptrq_value);
		ATF_CHECK_MSG(live[i].ptrq_value == expected[i],
		    "thread %c expected restored live VALID/%ju, "
		    "observed VALID/%ju", 'A' + i, (uintmax_t)expected[i],
		    (uintmax_t)live[i].ptrq_value);
	}
}
ATF_TC_CLEANUP(per_thread_residuals_survive_eviction, tc)
{

	reset_residual_test_hooks();
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, long_period_unmultiplexed);
	ATF_TP_ADD_TC(tp, long_period_under_mux);
	ATF_TP_ADD_TC(tp, residual_state_identity);
	ATF_TP_ADD_TC(tp, thread_residual_new_thread_starts_full_period);
	ATF_TP_ADD_TC(tp, thread_residual_thread_exit_reclaims_state);
	ATF_TP_ADD_TC(tp, per_thread_residuals_survive_eviction);

	return (atf_no_error());
}
