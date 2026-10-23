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

static void
require_hwpmc(void)
{

	if (pmc_init() != 0)
		atf_tc_skip("hwpmc(4) is not available");
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
	pthread_t thr;
	pmc_id_t id;
	uint64_t p0, p1, th0, th1;
	double secs;
	u_int i;
	int fd;
	bool ok;

	if (sysconf(_SC_NPROCESSORS_ONLN) <= TEST_CPU)
		atf_tc_skip("needs at least %d CPUs", TEST_CPU + 1);

	ok = false;
	for (i = 0; i < nitems(cycle_events); i++) {
		if (pmc_allocate(cycle_events[i], PMC_MODE_SS, 0, TEST_CPU,
		    &id, TEST_PERIOD) == 0) {
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
	ATF_REQUIRE(pmc_release(id) == 0);
	if (logpath != NULL)
		ATF_REQUIRE(pmc_close_logfile() == 0);
	(void)pmc_configure_logfile(-1);
	close(fd);

	secs = (t1.tv_sec - t0.tv_sec) + (t1.tv_usec - t0.tv_usec) / 1e6;
	*throttled = th1 - th0;
	return ((p1 - p0) / secs);
}

/* Allow one stop interrupt per tick and a margin for other PMCs. */
ATF_TC(rate_limited);
ATF_TC_HEAD(rate_limited, tc)
{
	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(rate_limited, tc)
{
	uint64_t throttled;
	double rate, bound;
	u_int oldrate, oldmin;
	int hz;

	require_hwpmc();
	oldrate = rate_get();
	hz = hz_get();
	oldmin = mincount_set(1);
	rate_set(10 * hz);

	rate = measure(&throttled);

	rate_set(oldrate);
	(void)mincount_set(oldmin);

	bound = 1.5 * 11 * hz;
	printf("hz %d, rate %.0f/s, bound %.0f/s, throttled %ju\n", hz, rate,
	    bound, (uintmax_t)throttled);
	ATF_CHECK_MSG(throttled > 0, "the PMC never reached the limit");
	ATF_CHECK_MSG(rate <= bound, "rate %.0f/s is above %.0f/s", rate,
	    bound);
}

/* Without throttling, the same PMC must exceed the rate limit. */
ATF_TC(rate_unlimited);
ATF_TC_HEAD(rate_unlimited, tc)
{
	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(rate_unlimited, tc)
{
	uint64_t throttled;
	double rate, bound;
	u_int oldrate, oldmin;
	int hz;

	require_hwpmc();
	oldrate = rate_get();
	hz = hz_get();
	oldmin = mincount_set(1);
	rate_set(0);

	rate = measure(&throttled);

	rate_set(oldrate);
	(void)mincount_set(oldmin);

	bound = 1.5 * 11 * hz;
	printf("hz %d, rate %.0f/s, throttled %ju\n", hz, rate,
	    (uintmax_t)throttled);
	ATF_CHECK_EQ_MSG(throttled, 0, "throttled %ju times",
	    (uintmax_t)throttled);
	ATF_CHECK_MSG(rate > bound, "rate %.0f/s is not above %.0f/s", rate,
	    bound);
}

/* A 1% CPU time limit must lower the sample rate and measured rate. */
ATF_TC(time_limited);
ATF_TC_HEAD(time_limited, tc)
{
	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(time_limited, tc)
{
	uint64_t lowered0, lowered1, throttled;
	double rate, bound;
	u_int oldrate, oldpct, oldmin, newrate;
	int hz;

	require_hwpmc();
	oldrate = rate_get();
	oldpct = pct_get();
	hz = hz_get();
	oldmin = mincount_set(1);
	rate_set(100 * hz);
	pct_set(1);

	lowered0 = stat_get("rate_lowered");
	rate = measure(&throttled);
	lowered1 = stat_get("rate_lowered");
	newrate = rate_get();

	pct_set(oldpct);
	rate_set(oldrate);
	(void)mincount_set(oldmin);

	bound = 1.5 * (newrate + hz);
	printf("hz %d, rate %.0f/s, new limit %u/s, lowered %ju\n", hz,
	    rate, newrate, (uintmax_t)(lowered1 - lowered0));
	ATF_CHECK_MSG(lowered1 > lowered0, "the rate was not lowered");
	ATF_CHECK_MSG(newrate < 100 * (u_int)hz,
	    "max_sample_rate %u was not lowered", newrate);
	ATF_CHECK_MSG(rate <= bound, "rate %.0f/s is above %.0f/s", rate,
	    bound);
}

/* DROPNOTIFY counts must approximate the intr_throttled delta. */
ATF_TC(drop_records);
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
	u_int oldrate, oldmin;
	void *parser;
	int fd, hz;

	require_hwpmc();
	oldrate = rate_get();
	hz = hz_get();
	oldmin = mincount_set(1);
	rate_set(10 * hz);

	(void)measure_log(&throttled, "drop.log", &id);

	rate_set(oldrate);
	(void)mincount_set(oldmin);

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

ATF_TP_ADD_TCS(tp)
{
	ATF_TP_ADD_TC(tp, rate_limited);
	ATF_TP_ADD_TC(tp, rate_unlimited);
	ATF_TP_ADD_TC(tp, time_limited);
	ATF_TP_ADD_TC(tp, drop_records);
	return (atf_no_error());
}
