/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 */

#ifndef _PMC_EXEC_CREDENTIAL_COMMON_H_
#define	_PMC_EXEC_CREDENTIAL_COMMON_H_

#include <sys/types.h>

#include <pmc.h>
#include <stdint.h>

#define	PMC_EXEC_CREDENTIAL_RESULT_VERSION	1U
#define	PMC_EXEC_CREDENTIAL_MAX_GROUPS		8
#define	PMC_EXEC_CREDENTIAL_RELAY_MODE		"--relay"

enum pmc_exec_credential_stage {
	PMC_EXEC_CREDENTIAL_STAGE_NONE,
	PMC_EXEC_CREDENTIAL_STAGE_CREDENTIALS,
	PMC_EXEC_CREDENTIAL_STAGE_GROUPS,
	PMC_EXEC_CREDENTIAL_STAGE_PMC_INIT,
	PMC_EXEC_CREDENTIAL_STAGE_READS,
	PMC_EXEC_CREDENTIAL_STAGE_DONE
};

struct pmc_exec_credential_result {
	uint32_t	per_version;
	uint32_t	per_stage;
	int		per_stage_errno;
	uid_t		per_ruid;
	uid_t		per_euid;
	uid_t		per_suid;
	gid_t		per_rgid;
	gid_t		per_egid;
	gid_t		per_sgid;
	int		per_ngroups;
	gid_t		per_groups[PMC_EXEC_CREDENTIAL_MAX_GROUPS];
	int		per_pmc_init_rc;
	int		per_pmc_init_errno;
	int		per_leader_rc;
	int		per_leader_errno;
	int		per_sibling_rc;
	int		per_sibling_errno;
	uint64_t	per_leader_value;
	uint64_t	per_sibling_value;
};

struct pmc_exec_target_command {
	pmc_id_t	petc_leader;
	pmc_id_t	petc_sibling;
};

struct pmc_exec_target_precheck {
	int		petp_leader_rc;
	int		petp_leader_errno;
	int		petp_sibling_rc;
	int		petp_sibling_errno;
	pmc_value_t	petp_leader_value;
	pmc_value_t	petp_sibling_value;
};

#endif /* _PMC_EXEC_CREDENTIAL_COMMON_H_ */
