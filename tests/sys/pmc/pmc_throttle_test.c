/*
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* Test per-tick sample rate and interrupt CPU time limits. */

#include <sys/param.h>
#include <sys/cpuset.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include <sys/wait.h>

#include <atf-c.h>
#include <errno.h>
#include <fcntl.h>
#include <pmc.h>
#include <pmclog.h>
#include <pthread.h>
#include <signal.h>
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

/* Read a kern.hwpmc u_int sysctl, or skip if absent. */
static u_int
hwpmc_get(const char *name)
{
	char full[64];
	size_t len;
	u_int v;

	snprintf(full, sizeof(full), "kern.hwpmc.%s", name);
	len = sizeof(v);
	if (sysctlbyname(full, &v, &len, NULL, 0) != 0)
		atf_tc_skip("%s is not available", full);
	return (v);
}

static int
hwpmc_write(const char *name, u_int v)
{
	char full[64];

	snprintf(full, sizeof(full), "kern.hwpmc.%s", name);
	return (sysctlbyname(full, NULL, NULL, &v, sizeof(v)));
}

static void
hwpmc_set(const char *name, u_int v)
{

	ATF_REQUIRE_MSG(hwpmc_write(name, v) == 0, "set kern.hwpmc.%s: %s",
	    name, strerror(errno));
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
	int fd;

	sv.mincount = hwpmc_get("mincount");
	sv.rate = hwpmc_get("max_sample_rate");
	sv.pct = hwpmc_get("cpu_time_max_percent");
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
		(void)hwpmc_write("max_sample_rate", sv.rate);
		(void)hwpmc_write("cpu_time_max_percent", sv.pct);
		(void)hwpmc_write("mincount", sv.mincount);
	}
	close(fd);
	(void)unlink(SAVE_FILE);
}

/* Save sysctls. Set mincount to 1 and the per-tick limit. */
static int
throttle_setup(u_int rate_per_tick)
{
	int hz;

	require_hwpmc();
	sysctls_save();
	hz = hz_get();
	hwpmc_set("mincount", 1);
	hwpmc_set("max_sample_rate", rate_per_tick * hz);
	return (hz);
}

/*
 * Measure interrupts per second and throttle count on TEST_CPU.
 * Optionally write logpath and return the PMC ID in *idp.
 */
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

static double
measure(uint64_t *throttled)
{

	return (measure_log(throttled, NULL, NULL));
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

	hz = throttle_setup(10);

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

	hz = throttle_setup(0);

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

	hz = throttle_setup(100);
	hwpmc_set("cpu_time_max_percent", 1);

	lowered0 = stat_get("rate_lowered");
	rate = measure(&throttled);
	lowered1 = stat_get("rate_lowered");
	newrate = hwpmc_get("max_sample_rate");

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
	int fd;

	(void)throttle_setup(10);

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
	/* pmc_stop() must log all final-tick drops. */
	ATF_CHECK_MSG(dropped == throttled,
	    "dropped %ju, throttled %ju", (uintmax_t)dropped,
	    (uintmax_t)throttled);
}

ATF_TC_CLEANUP(drop_records, tc)
{

	sysctls_restore();
}

/* Sum THROTTLED DROPNOTIFY counts for PMC id. */
static uint64_t
log_drops(const char *logpath, pmc_id_t id, uint64_t *nrec)
{
	struct pmclog_ev ev;
	uint64_t dropped;
	void *parser;
	int fd;

	fd = open(logpath, O_RDONLY);
	ATF_REQUIRE(fd >= 0);
	parser = pmclog_open(fd);
	ATF_REQUIRE(parser != NULL);
	dropped = *nrec = 0;
	while (pmclog_read(parser, &ev) == 0) {
		if (ev.pl_type != PMCLOG_TYPE_DROPNOTIFY ||
		    ev.pl_u.pl_dn.pl_pmcid != id ||
		    (ev.pl_u.pl_dn.pl_flags & PMCLOG_DROP_THROTTLED) == 0)
			continue;
		(*nrec)++;
		dropped += ev.pl_u.pl_dn.pl_count;
	}
	ATF_CHECK_MSG(ev.pl_state == PMCLOG_EOF, "log state %d",
	    ev.pl_state);
	pmclog_close(parser);
	close(fd);
	return (dropped);
}

/* Process PMCs must log all drops at context switches and exit. */
ATF_TC_WITH_CLEANUP(drop_records_process);
ATF_TC_HEAD(drop_records_process, tc)
{
	atf_tc_set_md_var(tc, "require.user", "root");
	atf_tc_set_md_var(tc, "timeout", "30");
}
ATF_TC_BODY(drop_records_process, tc)
{
	struct timespec t, t0;
	volatile uint64_t x;
	uint64_t dropped, nrec, th0, th1;
	pmc_id_t id;
	pid_t pid;
	int fd, i, pfd[2], status;
	char c;

	(void)throttle_setup(10);

	ATF_REQUIRE(pipe(pfd) == 0);
	pid = fork();
	ATF_REQUIRE(pid >= 0);
	if (pid == 0) {
		/* 300 bursts of 2 ms, with 1 ms sleeps. */
		pin(TEST_CPU);
		if (read(pfd[0], &c, 1) != 1)
			_exit(1);
		x = 1;
		for (i = 0; i < 300; i++) {
			clock_gettime(CLOCK_MONOTONIC, &t0);
			do {
				x = x * 6364136223846793005ULL + 1;
				clock_gettime(CLOCK_MONOTONIC, &t);
			} while ((t.tv_sec - t0.tv_sec) * 1000000000L +
			    (t.tv_nsec - t0.tv_nsec) < 2000000L);
			usleep(1000);
		}
		_exit(0);
	}

	for (i = 0; i < (int)nitems(cycle_events); i++) {
		if (pmc_allocate(cycle_events[i], PMC_MODE_TS, 0, PMC_CPU_ANY,
		    &id, TEST_PERIOD) == 0)
			break;
	}
	if (i == (int)nitems(cycle_events)) {
		(void)kill(pid, SIGKILL);
		(void)waitpid(pid, NULL, 0);
		atf_tc_skip("no cycle event for a process sampling PMC");
	}
	fd = open("dropp.log", O_WRONLY | O_CREAT | O_TRUNC, 0600);
	ATF_REQUIRE(fd >= 0);
	ATF_REQUIRE(pmc_configure_logfile(fd) == 0);
	ATF_REQUIRE(pmc_attach(id, pid) == 0);
	th0 = stat_get("intr_throttled");
	ATF_REQUIRE(pmc_start(id) == 0);
	ATF_REQUIRE(write(pfd[1], "g", 1) == 1);
	ATF_REQUIRE(waitpid(pid, &status, 0) == pid);
	th1 = stat_get("intr_throttled");
	(void)pmc_stop(id);
	ATF_REQUIRE(pmc_release(id) == 0);
	ATF_REQUIRE(pmc_close_logfile() == 0);
	(void)pmc_configure_logfile(-1);
	close(fd);
	close(pfd[0]);
	close(pfd[1]);

	sysctls_restore();

	dropped = log_drops("dropp.log", id, &nrec);
	printf("throttled %ju, records %ju, dropped %ju\n",
	    (uintmax_t)(th1 - th0), (uintmax_t)nrec, (uintmax_t)dropped);
	ATF_CHECK_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
	    "child status %d", status);
	ATF_CHECK_MSG(th1 - th0 > 0, "the PMC never reached the limit");
	/* An in-flight NMI can count one unlogged drop after exit. */
	ATF_CHECK_MSG(dropped <= th1 - th0 && dropped + 1 >= th1 - th0,
	    "dropped %ju, throttled %ju", (uintmax_t)dropped,
	    (uintmax_t)(th1 - th0));
}

ATF_TC_CLEANUP(drop_records_process, tc)
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

	hz = throttle_setup(100);
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

	hz = throttle_setup(10);
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
	ATF_TP_ADD_TC(tp, drop_records_process);
	ATF_TP_ADD_TC(tp, short_period);
	ATF_TP_ADD_TC(tp, flush_loop);
	return (atf_no_error());
}
