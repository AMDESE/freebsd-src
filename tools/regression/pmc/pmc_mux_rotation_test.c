/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Test PMU multiplex rotation policy.
 */

#include <sys/types.h>
#include <sys/sysctl.h>
#include <sys/time.h>

#include <err.h>
#include <errno.h>
#include <pmc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define	MAX_PER_GROUP	8

static const char *event_pool[] = {
	"instructions",
	"unhalted-cycles",
	"branches",
	"branch-misses",
	"cache-references",
	"cache-misses",
	"l1d-loads",
	"l1d-load-misses",
	"l2-cache-references",
	"l2-cache-misses",
	"dispatch-stalls",
	"fp-retired",
	"branches-retired",
	"de-no-dispatch-per-slot",
	"ls_alloc_mab_count",
	"ls_not_halted_cyc",
	"ls_dispatch.all",
	"ls_mab_alloc.ls",
	"ls_mab_alloc.hwpf",
	"ls_mab_alloc.all",
	"ls_int_taken",
	"ls_stlf",
	/* Native Zen event names. */
	"ex_ret_brn",
	"ex_ret_brn_misp",
	"ex_ret_brn_tkn",
	"ex_ret_near_ret",
	"ex_ret_cond",
	"ex_ret_ind_brch_instr",
	"bp_l2_btb_correct",
	"bp_dyn_ind_pred",
	"bp_de_redirect",
	"ls_mab_alloc.load_store_allocations",
	"ls_dmnd_fills_from_sys.local_l2",
	"ls_any_fills_from_sys.local_l2",
};
#define	POOL_SIZE	(int)(sizeof(event_pool) / sizeof(event_pool[0]))

#define	MUX_PERIOD_MS	10

struct grp {
	uint32_t	gid;
	int		nevents;
	int		committed;
	pmc_id_t	ids[MAX_PER_GROUP];
};

static int
is_amd(void)
{
	char buf[64];
	size_t s = sizeof(buf);

	if (sysctlbyname("kern.hwpmc.cpuid", buf, &s, NULL, 0) != 0)
		return (0);
	return (strstr(buf, "AuthenticAMD") != NULL ||
	    strstr(buf, "HygonGenuine") != NULL);
}

/* Return count of available core hardware counters. */
static int
probe_core_pmcs(void)
{
	pmc_id_t ids[64];
	int n = 0;

	while (n < (int)(sizeof(ids) / sizeof(ids[0]))) {
		if (pmc_allocate("instructions", PMC_MODE_TC, 0,
		    PMC_CPU_ANY, &ids[n], 0) < 0)
			break;
		n++;
	}
	for (int i = 0; i < n; i++)
		(void)pmc_release(ids[i]);
	return (n);
}

static void
busy_for(uint64_t ns)
{
	struct timespec t0, now;
	volatile uint64_t spin = 0;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (;;) {
		for (int k = 0; k < 200000; k++)
			spin++;
		clock_gettime(CLOCK_MONOTONIC, &now);
		uint64_t elapsed = (uint64_t)(now.tv_sec - t0.tv_sec) *
		    1000000000ULL + (uint64_t)(now.tv_nsec - t0.tv_nsec);
		if (elapsed >= ns)
			return;
	}
}

static int
set_mux_period(int new_ms, int *saved)
{
	size_t s = sizeof(*saved);

	*saved = -1;
	if (sysctlbyname("kern.hwpmc.mux_period_ms", saved, &s,
	    &new_ms, sizeof(new_ms)) != 0) {
		*saved = -1;
		return (-1);
	}
	return (0);
}

static void
restore_mux_period(int saved)
{
	if (saved < 0)
		return;
	(void)sysctlbyname("kern.hwpmc.mux_period_ms", NULL, NULL,
	    &saved, sizeof(saved));
}

/*
 * Build a group of n distinct events from the pool.
 */
static int
build_group(struct grp *g, int n, int *cursor, int mux)
{
	memset(g, 0, sizeof(*g));
	if (pmc_group_create(&g->gid) < 0) {
		warn("pmc_group_create");
		return (-1);
	}
	while (g->nevents < n && *cursor < POOL_SIZE) {
		uint32_t flags = 0;
		pmc_id_t id;
		const char *ev = event_pool[(*cursor)++];

		if (g->nevents == 0 && mux)
			flags |= PMC_F_GROUP_MUX;
		if (pmc_allocate_group(ev, PMC_MODE_TC, flags,
		    PMC_CPU_ANY, &id, 0) < 0)
			continue;	/* Event not supported on this model. */
		if (pmc_group_add(g->gid, id, g->nevents == 0) < 0) {
			(void)pmc_release(id);
			continue;
		}
		g->ids[g->nevents++] = id;
	}
	return (g->nevents == n ? 0 : -1);
}

static void
destroy_group(struct grp *g)
{
	if (g->nevents == 0)
		return;
	if (g->committed)
		(void)pmc_release(g->ids[0]);
	else {
		for (int i = 0; i < g->nevents; i++)
			(void)pmc_release(g->ids[i]);
	}
	g->nevents = 0;
	g->committed = 0;
}

/* Commit, attach, and start group. */
static int
launch_group(struct grp *g, const char *tag)
{
	if (pmc_group_commit(g->gid) < 0) {
		warn("pmc_group_commit %s", tag);
		return (-1);
	}
	g->committed = 1;
	if (pmc_attach(g->ids[0], getpid()) < 0) {
		warn("pmc_attach %s", tag);
		return (-1);
	}
	if (pmc_start(g->ids[0]) < 0) {
		warn("pmc_start %s", tag);
		return (-1);
	}
	return (0);
}

static int
grp_times(struct grp *g, uint64_t *enabled, uint64_t *running)
{
	struct pmc_group_times t;
	uint32_t n = 0;

	memset(&t, 0, sizeof(t));
	if (pmc_group_read(g->ids[0], &n, NULL, &t) < 0)
		return (-1);
	*enabled = t.pgt_enabled;
	*running = t.pgt_running;
	return (0);
}

/*
 * Return 1 if rotation thread exists, 0 if not, -1 on error.
 */
static int
rot_thread_visible(void)
{
	FILE *fp;
	char line[512];
	int found = 0, sawany = 0;

	fp = popen("ps -axH 2>/dev/null", "r");
	if (fp == NULL)
		return (-1);
	while (fgets(line, sizeof(line), fp) != NULL) {
		sawany = 1;
		if (strstr(line, "pmu_rot") != NULL)
			found = 1;
	}
	(void)pclose(fp);
	return (sawany ? found : -1);
}

/*
 * Calculate group size for three oversubscribing groups.
 */
static int
mux_group_size(int core)
{
	int s = core / 2;

	if (s < 1 || s > MAX_PER_GROUP || 3 * s <= core)
		return (-1);
	return (s);
}

/*
 * Test 1: verify equal execution ratios across three rotating groups.
 */
static int
test_fairness(int core)
{
	struct grp g[3];
	uint64_t en1[3], ru1[3], en2[3], ru2[3];
	double ratio[3], lo, hi;
	int cursor = 0, i, rc = 1, s;

	s = mux_group_size(core);
	if (s < 0) {
		printf("SKIP fairness: cannot oversubscribe on %d counters\n",
		    core);
		return (77);
	}

	memset(g, 0, sizeof(g));
	for (i = 0; i < 3; i++) {
		if (build_group(&g[i], s, &cursor, 1) < 0 ||
		    launch_group(&g[i], "fairness") < 0) {
			printf("SKIP fairness: could not build/launch "
			    "group %d\n", i);
			rc = 77;
			goto out;
		}
	}

	busy_for(500ULL * 1000 * 1000);
	for (i = 0; i < 3; i++)
		if (grp_times(&g[i], &en1[i], &ru1[i]) < 0) {
			warn("pmc_group_read");
			goto out;
		}
	busy_for(3000ULL * 1000 * 1000);
	for (i = 0; i < 3; i++)
		if (grp_times(&g[i], &en2[i], &ru2[i]) < 0) {
			warn("pmc_group_read");
			goto out;
		}

	lo = 2.0;
	hi = 0.0;
	for (i = 0; i < 3; i++) {
		uint64_t den = en2[i] - en1[i], drn = ru2[i] - ru1[i];

		if (den == 0) {
			fprintf(stderr, "FAIL fairness: group %d accrued no "
			    "enabled time\n", i);
			goto out;
		}
		ratio[i] = (double)drn / (double)den;
		printf("  fairness: group %d running/enabled = %.4f\n",
		    i, ratio[i]);
		if (ratio[i] <= 0.0) {
			fprintf(stderr, "FAIL fairness: group %d got no "
			    "hardware time\n", i);
			goto out;
		}
		if (ratio[i] < lo)
			lo = ratio[i];
		if (ratio[i] > hi)
			hi = ratio[i];
	}
	printf("  fairness: max/min ratio = %.4f (limit 1.25)\n", hi / lo);
	if (hi / lo > 1.25) {
		fprintf(stderr, "FAIL fairness: ratio spread %.4f > 1.25\n",
		    hi / lo);
		goto out;
	}
	rc = 0;
out:
	for (i = 0; i < 3; i++)
		destroy_group(&g[i]);
	return (rc);
}

/*
 * Test 2: verify non-multiplex groups are never evicted.
 */
static int
test_pinned(int core)
{
	struct grp p, m1, m2;
	uint64_t pen1, pru1, pen2, pru2, en, ru1a, ru2a, ru1b, ru2b;
	double pinned_ratio;
	int cursor = 0, rc = 1, s;

	s = mux_group_size(core);
	if (s < 0) {
		printf("SKIP pinned: cannot oversubscribe on %d counters\n",
		    core);
		return (77);
	}

	memset(&p, 0, sizeof(p));
	memset(&m1, 0, sizeof(m1));
	memset(&m2, 0, sizeof(m2));
	if (build_group(&p, s, &cursor, 0) < 0 ||
	    launch_group(&p, "pinned") < 0 ||
	    build_group(&m1, s, &cursor, 1) < 0 ||
	    launch_group(&m1, "mux1") < 0 ||
	    build_group(&m2, s, &cursor, 1) < 0 ||
	    launch_group(&m2, "mux2") < 0) {
		printf("SKIP pinned: could not build/launch groups\n");
		rc = 77;
		goto out;
	}

	busy_for(500ULL * 1000 * 1000);
	if (grp_times(&p, &pen1, &pru1) < 0 ||
	    grp_times(&m1, &en, &ru1a) < 0 ||
	    grp_times(&m2, &en, &ru1b) < 0) {
		warn("pmc_group_read");
		goto out;
	}
	busy_for(3000ULL * 1000 * 1000);
	if (grp_times(&p, &pen2, &pru2) < 0 ||
	    grp_times(&m1, &en, &ru2a) < 0 ||
	    grp_times(&m2, &en, &ru2b) < 0) {
		warn("pmc_group_read");
		goto out;
	}

	if (pen2 <= pen1) {
		fprintf(stderr, "FAIL pinned: no enabled time accrued\n");
		goto out;
	}
	pinned_ratio = (double)(pru2 - pru1) / (double)(pen2 - pen1);
	printf("  pinned: non-MUX running/enabled = %.4f, "
	    "MUX deltas = %ju / %ju\n", pinned_ratio,
	    (uintmax_t)(ru2a - ru1a), (uintmax_t)(ru2b - ru1b));
	if (pinned_ratio < 0.99) {
		fprintf(stderr, "FAIL pinned: non-MUX group lost hardware "
		    "time (ratio %.4f < 0.99) -- it was evicted\n",
		    pinned_ratio);
		goto out;
	}
	if (ru2a <= ru1a || ru2b <= ru1b) {
		fprintf(stderr, "FAIL pinned: a MUX group made no progress "
		    "alongside the pinned group\n");
		goto out;
	}
	rc = 0;
out:
	destroy_group(&m2);
	destroy_group(&m1);
	destroy_group(&p);
	return (rc);
}

/*
 * Test 3: verify multi-victim escalation when large groups wait.
 */
static int
test_escalation(int core)
{
	struct grp a, b, c;
	uint64_t en, ru;
	int cursor = 0, rc = 1, i, sa, sc;

	sa = core / 2;
	sc = core - sa + 1;
	if (sa < 1 || sc > MAX_PER_GROUP || sc > core || sa + sa > core) {
		printf("SKIP escalation: no valid sizes on %d counters\n",
		    core);
		return (77);
	}

	memset(&a, 0, sizeof(a));
	memset(&b, 0, sizeof(b));
	memset(&c, 0, sizeof(c));
	if (build_group(&a, sa, &cursor, 1) < 0 ||
	    launch_group(&a, "escA") < 0 ||
	    build_group(&b, sa, &cursor, 1) < 0 ||
	    launch_group(&b, "escB") < 0 ||
	    build_group(&c, sc, &cursor, 1) < 0 ||
	    launch_group(&c, "escC") < 0) {
		printf("SKIP escalation: could not build/launch groups\n");
		rc = 77;
		goto out;
	}

	printf("  escalation: A=%d B=%d C=%d on %d counters\n",
	    sa, sa, sc, core);
	for (i = 0; i < 50; i++) {
		busy_for(200ULL * 1000 * 1000);
		if (grp_times(&c, &en, &ru) < 0) {
			warn("pmc_group_read");
			goto out;
		}
		if (ru > 0)
			break;
	}
	printf("  escalation: C running=%ju enabled=%ju after %d ms\n",
	    (uintmax_t)ru, (uintmax_t)en, (i < 50 ? i + 1 : 50) * 200);
	if (ru == 0) {
		fprintf(stderr, "FAIL escalation: C never got hardware time "
		    "within 10 s -- multi-victim escalation broken\n");
		goto out;
	}
	rc = 0;
out:
	destroy_group(&c);
	destroy_group(&b);
	destroy_group(&a);
	return (rc);
}

/*
 * Test 4: verify rotation thread idle stop and restart on new demand.
 */
static int
test_selfstop_rekick(int core)
{
	struct grp m1, m2, m3;
	uint64_t en, ru_a, ru_b, ru;
	int cursor = 0, rc = 1, vis, i, s;

	s = core / 2 + 1;
	if (s > MAX_PER_GROUP || 2 * s <= core || s > core) {
		printf("SKIP self-stop: no valid sizes on %d counters\n",
		    core);
		return (77);
	}

	memset(&m1, 0, sizeof(m1));
	memset(&m2, 0, sizeof(m2));
	memset(&m3, 0, sizeof(m3));
	if (build_group(&m1, s, &cursor, 1) < 0 ||
	    launch_group(&m1, "ss1") < 0 ||
	    build_group(&m2, s, &cursor, 1) < 0 ||
	    launch_group(&m2, "ss2") < 0) {
		printf("SKIP self-stop: could not build/launch groups\n");
		rc = 77;
		goto out;
	}

	busy_for(500ULL * 1000 * 1000);
	if (grp_times(&m1, &en, &ru_a) < 0 ||
	    grp_times(&m2, &en, &ru_b) < 0) {
		warn("pmc_group_read");
		goto out;
	}
	if (ru_a == 0 || ru_b == 0) {
		fprintf(stderr, "FAIL self-stop: rotation between two "
		    "mutually exclusive MUX groups is not happening\n");
		goto out;
	}
	vis = rot_thread_visible();
	if (vis < 0) {
		printf("SKIP self-stop: ps -axH unavailable, cannot "
		    "observe kthreads\n");
		rc = 77;
		goto out;
	}
	if (vis == 0) {
		fprintf(stderr, "FAIL self-stop: groups rotate but no "
		    "pmu_rot thread is visible in ps -axH\n");
		goto out;
	}

	/* Thread must stop when remaining groups fit. */
	destroy_group(&m1);
	usleep(600 * 1000);
	vis = rot_thread_visible();
	if (vis != 0) {
		fprintf(stderr, "FAIL self-stop: pmu_rot thread still "
		    "alive 600 ms after every group fit\n");
		goto out;
	}
	printf("  self-stop: kthread exited after the survivors fit\n");

	/* Thread must restart when a new group is deferred. */
	if (build_group(&m3, s, &cursor, 1) < 0 ||
	    launch_group(&m3, "ss3") < 0) {
		printf("SKIP self-stop: could not build/launch third group\n");
		rc = 77;
		goto out;
	}
	vis = 0;
	for (i = 0; i < 3 && vis <= 0; i++) {
		vis = rot_thread_visible();
		if (vis <= 0)
			usleep(MUX_PERIOD_MS * 1000);
	}
	if (vis <= 0) {
		fprintf(stderr, "FAIL re-kick: no pmu_rot thread within two "
		    "periods of starting a deferred group\n");
		goto out;
	}
	busy_for(2000ULL * 1000 * 1000);
	if (grp_times(&m3, &en, &ru) < 0) {
		warn("pmc_group_read");
		goto out;
	}
	printf("  re-kick: kthread respawned, new group running=%ju\n",
	    (uintmax_t)ru);
	if (ru == 0) {
		fprintf(stderr, "FAIL re-kick: respawned rotation never "
		    "placed the new group\n");
		goto out;
	}
	rc = 0;
out:
	destroy_group(&m3);
	destroy_group(&m2);
	destroy_group(&m1);
	return (rc);
}

int
main(void)
{
	int core, saved_period;
	int rc, failed = 0, passed = 0, skipped = 0;

	if (pmc_init() < 0)
		err(1, "pmc_init");
	if (!is_amd()) {
		printf("SKIP: non-AMD CPU\n");
		return (77);
	}
	core = probe_core_pmcs();
	if (core < 4) {
		printf("SKIP: only %d core PMCs available\n", core);
		return (77);
	}

	if (set_mux_period(MUX_PERIOD_MS, &saved_period) == 0)
		printf("set kern.hwpmc.mux_period_ms = %d (was %d)\n",
		    MUX_PERIOD_MS, saved_period);
	else
		printf("note: could not write kern.hwpmc.mux_period_ms; "
		    "using kernel default\n");

	static const struct {
		const char *name;
		int (*fn)(int);
	} tests[] = {
		{ "fairness",		test_fairness },
		{ "pinned",		test_pinned },
		{ "escalation",		test_escalation },
		{ "self-stop/re-kick",	test_selfstop_rekick },
	};
	for (size_t t = 0; t < sizeof(tests) / sizeof(tests[0]); t++) {
		printf("==> %s\n", tests[t].name);
		rc = tests[t].fn(core);
		if (rc == 0) {
			printf("PASS: %s\n", tests[t].name);
			passed++;
		} else if (rc == 77) {
			skipped++;
		} else {
			printf("FAIL: %s\n", tests[t].name);
			failed++;
		}
	}

	restore_mux_period(saved_period);

	printf("\npmc_mux_rotation_test: %d passed, %d failed, %d skipped "
	    "(%d HW core counters, %d-ms rotation floor)\n",
	    passed, failed, skipped, core, MUX_PERIOD_MS);
	if (failed > 0)
		return (1);
	if (passed == 0)
		return (77);
	return (0);
}
