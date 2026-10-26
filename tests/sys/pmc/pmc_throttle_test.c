/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

/* Test per-tick sample rate and interrupt CPU time limits. */

#include <sys/param.h>
#include <sys/cpuset.h>
#include <sys/sysctl.h>
#include <sys/time.h>

#include <atf-c.h>
#include <errno.h>
#include <fcntl.h>
#include <pmc.h>
#include <pmclog.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define	TEST_CPU	1
#define	TEST_PERIOD	2000
#define	TEST_SECONDS	2

static const char *cycle_events[] = {
	"ls_not_halted_cyc",	/* AMD Zen */
	"unhalted-cycles",	/* Generic alias */
	"cpu_clk_unhalted.thread_p",	/* Intel */
};

static atomic_bool spin_stop;
static atomic_bool flush_stop;
static atomic_ulong nflush;
static bool use_flush;
static int test_period = TEST_PERIOD;
static double test_secs;

static void
require_hwpmc(void)
{

	if (pmc_init() != 0)
		atf_tc_skip("hwpmc(4) is not available");
	if (sysconf(_SC_NPROCESSORS_ONLN) <= TEST_CPU)
		atf_tc_skip("needs at least %d CPUs", TEST_CPU + 1);
}

static void
pin(int cpu)
{
	cpuset_t set;

	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	ATF_REQUIRE(cpuset_setaffinity(CPU_LEVEL_WHICH, CPU_WHICH_TID, -1,
	    sizeof(set), &set) == 0);
}

static void *
spin_main(void *arg __unused)
{
	volatile uint64_t x;

	pin(TEST_CPU);
	x = 1;
	while (!atomic_load(&spin_stop))
		x = x * 6364136223846793005ULL + 1;
	return (NULL);
}

static void *
flush_main(void *arg __unused)
{

	while (!atomic_load(&flush_stop)) {
		if (pmc_flush_logfile() == 0)
			atomic_fetch_add(&nflush, 1);
	}
	return (NULL);
}

static uint64_t
stat_get(const char *name)
{
	char full[128];
	uint64_t v;
	size_t len;

	snprintf(full, sizeof(full), "kern.hwpmc.stats.%s", name);
	len = sizeof(v);
	ATF_REQUIRE_MSG(sysctlbyname(full, &v, &len, NULL, 0) == 0,
	    "sysctl %s: %s", full, strerror(errno));
	return (v);
}

static u_int
rate_get(void)
{
	u_int v;
	size_t len;

	len = sizeof(v);
	if (sysctlbyname("kern.hwpmc.max_sample_rate", &v, &len, NULL,
	    0) != 0)
		atf_tc_skip("kern.hwpmc.max_sample_rate is not available");
	return (v);
}

static void
rate_set(u_int v)
{

	ATF_REQUIRE_MSG(sysctlbyname("kern.hwpmc.max_sample_rate", NULL, NULL,
	    &v, sizeof(v)) == 0, "set max_sample_rate: %s", strerror(errno));
}

static u_int
pct_get(void)
{
	u_int v;
	size_t len;

	len = sizeof(v);
	if (sysctlbyname("kern.hwpmc.cpu_time_max_percent", &v, &len, NULL,
	    0) != 0)
		atf_tc_skip("kern.hwpmc.cpu_time_max_percent is not available");
	return (v);
}

static void
pct_set(u_int v)
{

	ATF_REQUIRE_MSG(sysctlbyname("kern.hwpmc.cpu_time_max_percent", NULL,
	    NULL, &v, sizeof(v)) == 0, "set cpu_time_max_percent: %s",
	    strerror(errno));
}

static int
hz_get(void)
{
	struct clockinfo ci;
	size_t len;

	len = sizeof(ci);
	ATF_REQUIRE(sysctlbyname("kern.clockrate", &ci, &len, NULL, 0) == 0);
	return (ci.hz);
}

static u_int
mincount_set(u_int v)
{
	u_int old;
	size_t len;

	len = sizeof(old);
	ATF_REQUIRE(sysctlbyname("kern.hwpmc.mincount", &old, &len, &v,
	    sizeof(v)) == 0);
	return (old);
}

/* Save sysctls in a file for cleanup in a separate ATF process. */
#define	SAVE_FILE	"sysctl.saved"

struct saved_sysctls {
	u_int	rate;
	u_int	pct;
	u_int	mincount;
};

static void
sysctls_save(void)
{
	struct saved_sysctls sv;
	size_t len;
	int fd;

	len = sizeof(sv.mincount);
	ATF_REQUIRE(sysctlbyname("kern.hwpmc.mincount", &sv.mincount, &len,
	    NULL, 0) == 0);
	sv.rate = rate_get();
	sv.pct = pct_get();
	fd = open(SAVE_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0600);
	ATF_REQUIRE(fd >= 0);
	ATF_REQUIRE(write(fd, &sv, sizeof(sv)) == sizeof(sv));
	close(fd);
}

static void
sysctls_restore(void)
{
	struct saved_sysctls sv;
	int fd;

	fd = open(SAVE_FILE, O_RDONLY);
	if (fd < 0)
		return;
	if (read(fd, &sv, sizeof(sv)) == sizeof(sv)) {
		(void)sysctlbyname("kern.hwpmc.max_sample_rate", NULL, NULL,
		    &sv.rate, sizeof(sv.rate));
		(void)sysctlbyname("kern.hwpmc.cpu_time_max_percent", NULL,
		    NULL, &sv.pct, sizeof(sv.pct));
		(void)sysctlbyname("kern.hwpmc.mincount", NULL, NULL,
		    &sv.mincount, sizeof(sv.mincount));
	}
	close(fd);
	(void)unlink(SAVE_FILE);
}

/* Measure interrupts per second on TEST_CPU and report *throttled. */
static double measure_log(uint64_t *throttled, const char *logpath,
    pmc_id_t *idp);

static double
measure(uint64_t *throttled)
{

	return (measure_log(throttled, NULL, NULL));
}

/* Optionally write logpath and return the PMC ID in *idp. */
static double
measure_log(uint64_t *throttled, const char *logpath, pmc_id_t *idp)
{
	struct timeval t0, t1;
	pthread_t fthr, thr;
	pmc_id_t id;
	uint64_t p0, p1, th0, th1;
	double secs;
	u_int i;
	int fd;
	bool ok;

	ok = false;
	for (i = 0; i < nitems(cycle_events); i++) {
		if (pmc_allocate(cycle_events[i], PMC_MODE_SS, 0, TEST_CPU,
		    &id, test_period) == 0) {
			ok = true;
			break;
		}
	}
	if (!ok)
		atf_tc_skip("no cycle event for a system sampling PMC");

	if (logpath != NULL)
		fd = open(logpath, O_WRONLY | O_CREAT | O_TRUNC, 0600);
	else
		fd = open("/dev/null", O_WRONLY);
	ATF_REQUIRE(fd >= 0);
	ATF_REQUIRE(pmc_configure_logfile(fd) == 0);
	if (idp != NULL)
		*idp = id;

	atomic_store(&spin_stop, false);
	ATF_REQUIRE(pthread_create(&thr, NULL, spin_main, NULL) == 0);
	usleep(100000);
	atomic_store(&flush_stop, false);
	atomic_store(&nflush, 0);
	if (use_flush)
		ATF_REQUIRE(pthread_create(&fthr, NULL, flush_main, NULL) == 0);

	p0 = stat_get("intr_processed");
	th0 = stat_get("intr_throttled");
	gettimeofday(&t0, NULL);
	ATF_REQUIRE(pmc_start(id) == 0);
	sleep(TEST_SECONDS);
	ATF_REQUIRE(pmc_stop(id) == 0);
	gettimeofday(&t1, NULL);
	p1 = stat_get("intr_processed");
	th1 = stat_get("intr_throttled");

	atomic_store(&spin_stop, true);
	pthread_join(thr, NULL);
	if (use_flush) {
		atomic_store(&flush_stop, true);
		pthread_join(fthr, NULL);
	}
	ATF_REQUIRE(pmc_release(id) == 0);
	if (logpath != NULL)
		ATF_REQUIRE(pmc_close_logfile() == 0);
	(void)pmc_configure_logfile(-1);
	close(fd);

	secs = (t1.tv_sec - t0.tv_sec) + (t1.tv_usec - t0.tv_usec) / 1e6;
	test_secs = secs;
	*throttled = th1 - th0;
	return ((p1 - p0) / secs);
}

/* Allow one stop interrupt per tick and a margin for other PMCs. */
ATF_TC_WITH_CLEANUP(rate_limited);
ATF_TC_HEAD(rate_limited, tc)
{
	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(rate_limited, tc)
{
	uint64_t throttled;
	double rate, bound;
	int hz;

	require_hwpmc();
	sysctls_save();
	hz = hz_get();
	(void)mincount_set(1);
	rate_set(10 * hz);

	rate = measure(&throttled);

	sysctls_restore();

	bound = 1.5 * 11 * hz;
	printf("hz %d, rate %.0f/s, bound %.0f/s, throttled %ju\n", hz, rate,
	    bound, (uintmax_t)throttled);
	ATF_CHECK_MSG(throttled > 0, "the PMC never reached the limit");
	ATF_CHECK_MSG(rate <= bound, "rate %.0f/s is above %.0f/s", rate,
	    bound);
}

ATF_TC_CLEANUP(rate_limited, tc)
{

	sysctls_restore();
}

/* Without throttling, the same PMC must exceed the rate limit. */
ATF_TC_WITH_CLEANUP(rate_unlimited);
ATF_TC_HEAD(rate_unlimited, tc)
{
	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(rate_unlimited, tc)
{
	uint64_t throttled;
	double rate, bound;
	int hz;

	require_hwpmc();
	sysctls_save();
	hz = hz_get();
	(void)mincount_set(1);
	rate_set(0);

	rate = measure(&throttled);

	sysctls_restore();

	bound = 1.5 * 11 * hz;
	printf("hz %d, rate %.0f/s, throttled %ju\n", hz, rate,
	    (uintmax_t)throttled);
	ATF_CHECK_EQ_MSG(throttled, 0, "throttled %ju times",
	    (uintmax_t)throttled);
	ATF_CHECK_MSG(rate > bound, "rate %.0f/s is not above %.0f/s", rate,
	    bound);
}

ATF_TC_CLEANUP(rate_unlimited, tc)
{

	sysctls_restore();
}

/* A 1% CPU time limit must lower the sample rate and measured rate. */
ATF_TC_WITH_CLEANUP(time_limited);
ATF_TC_HEAD(time_limited, tc)
{
	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(time_limited, tc)
{
	uint64_t lowered0, lowered1, throttled;
	double rate, bound;
	u_int newrate;
	int hz;

	require_hwpmc();
	sysctls_save();
	hz = hz_get();
	(void)mincount_set(1);
	rate_set(100 * hz);
	pct_set(1);

	lowered0 = stat_get("rate_lowered");
	rate = measure(&throttled);
	lowered1 = stat_get("rate_lowered");
	newrate = rate_get();

	sysctls_restore();

	bound = 1.5 * (newrate + hz);
	printf("hz %d, rate %.0f/s, new limit %u/s, lowered %ju\n", hz,
	    rate, newrate, (uintmax_t)(lowered1 - lowered0));
	ATF_CHECK_MSG(lowered1 > lowered0, "the rate was not lowered");
	ATF_CHECK_MSG(newrate < 100 * (u_int)hz,
	    "max_sample_rate %u was not lowered", newrate);
	ATF_CHECK_MSG(rate <= bound, "rate %.0f/s is above %.0f/s", rate,
	    bound);
}

ATF_TC_CLEANUP(time_limited, tc)
{

	sysctls_restore();
}

/* DROPNOTIFY counts must approximate the intr_throttled delta. */
ATF_TC_WITH_CLEANUP(drop_records);
ATF_TC_HEAD(drop_records, tc)
{
	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(drop_records, tc)
{
	struct pmclog_ev ev;
	uint64_t throttled, dropped, nrec, other;
	pmc_id_t id;
	void *parser;
	int fd, hz;

	require_hwpmc();
	sysctls_save();
	hz = hz_get();
	(void)mincount_set(1);
	rate_set(10 * hz);

	(void)measure_log(&throttled, "drop.log", &id);

	sysctls_restore();

	fd = open("drop.log", O_RDONLY);
	ATF_REQUIRE(fd >= 0);
	parser = pmclog_open(fd);
	ATF_REQUIRE(parser != NULL);
	dropped = nrec = other = 0;
	while (pmclog_read(parser, &ev) == 0) {
		if (ev.pl_type != PMCLOG_TYPE_DROPNOTIFY)
			continue;
		nrec++;
		if (ev.pl_u.pl_dn.pl_pmcid != id ||
		    ev.pl_u.pl_dn.pl_cpu != TEST_CPU ||
		    (ev.pl_u.pl_dn.pl_flags & PMCLOG_DROP_THROTTLED) == 0) {
			other++;
			continue;
		}
		dropped += ev.pl_u.pl_dn.pl_count;
	}
	ATF_CHECK_MSG(ev.pl_state == PMCLOG_EOF, "log state %d",
	    ev.pl_state);
	pmclog_close(parser);
	close(fd);

	printf("throttled %ju, records %ju, dropped %ju, other %ju\n",
	    (uintmax_t)throttled, (uintmax_t)nrec, (uintmax_t)dropped,
	    (uintmax_t)other);
	ATF_CHECK_MSG(throttled > 0, "the PMC never reached the limit");
	ATF_CHECK_MSG(nrec > 0, "no DROPNOTIFY record in the log");
	ATF_CHECK_EQ_MSG(other, 0, "%ju records with wrong fields",
	    (uintmax_t)other);
	/* Allow for unlogged drops after the last tick. */
	ATF_CHECK_MSG(dropped <= throttled && dropped + 10 >= throttled,
	    "dropped %ju, throttled %ju", (uintmax_t)dropped,
	    (uintmax_t)throttled);
}

ATF_TC_CLEANUP(drop_records, tc)
{

	sysctls_restore();
}

/* Short periods must cause one throttle per tick, not two. */
ATF_TC_WITH_CLEANUP(short_period);
ATF_TC_HEAD(short_period, tc)
{
	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(short_period, tc)
{
	uint64_t throttled;
	double per_tick, rate;
	int hz;

	require_hwpmc();
	sysctls_save();
	hz = hz_get();
	(void)mincount_set(1);
	rate_set(100 * hz);
	test_period = 100;

	rate = measure(&throttled);

	sysctls_restore();

	per_tick = throttled / test_secs / hz;
	printf("hz %d, rate %.0f/s, throttled %ju, %.2f each tick\n", hz,
	    rate, (uintmax_t)throttled, per_tick);
	ATF_CHECK_MSG(throttled > 0, "the PMC never reached the limit");
	ATF_CHECK_MSG(per_tick < 1.5, "%.2f throttles in each tick",
	    per_tick);
}

ATF_TC_CLEANUP(short_period, tc)
{

	sysctls_restore();
}

/* Log flushes must not reset the tick limit. */
ATF_TC_WITH_CLEANUP(flush_loop);
ATF_TC_HEAD(flush_loop, tc)
{
	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(flush_loop, tc)
{
	uint64_t throttled;
	double rate, bound;
	int hz;

	require_hwpmc();
	sysctls_save();
	hz = hz_get();
	(void)mincount_set(1);
	rate_set(10 * hz);
	use_flush = true;

	rate = measure(&throttled);

	sysctls_restore();

	bound = 1.5 * 11 * hz;
	printf("hz %d, rate %.0f/s, bound %.0f/s, flushes %lu, "
	    "throttled %ju\n", hz, rate, bound, atomic_load(&nflush),
	    (uintmax_t)throttled);
	ATF_CHECK_MSG(atomic_load(&nflush) > 0, "no flush succeeded");
	ATF_CHECK_MSG(rate <= bound, "rate %.0f/s is above %.0f/s", rate,
	    bound);
}

ATF_TC_CLEANUP(flush_loop, tc)
{

	sysctls_restore();
}

ATF_TP_ADD_TCS(tp)
{
	ATF_TP_ADD_TC(tp, rate_limited);
	ATF_TP_ADD_TC(tp, rate_unlimited);
	ATF_TP_ADD_TC(tp, time_limited);
	ATF_TP_ADD_TC(tp, drop_records);
	ATF_TP_ADD_TC(tp, short_period);
	ATF_TP_ADD_TC(tp, flush_loop);
	return (atf_no_error());
}
