#!/usr/bin/env python3
"""Static conformance checks for the versioned Praxis learning contract.

This intentionally uses only Python's standard library so it can run before
the Rust toolchain or zome WASM artifacts are available in CI.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTRACTS = ROOT / "crates/praxis-core/src/contracts.rs"
COORDINATOR = ROOT / "zomes/learning_zome/coordinator/src/lib.rs"
INTEGRATION_COORDINATOR = ROOT / "zomes/integration_zome/coordinator/src/lib.rs"
ADAPTIVE_COORDINATOR = ROOT / "zomes/adaptive_zome/coordinator/src/lib.rs"
CREDENTIAL_COORDINATOR = ROOT / "zomes/credential_zome/coordinator/src/lib.rs"
COURSES_PAGE = ROOT / "apps/leptos/src/pages/courses.rs"
STUDY_PAGE = ROOT / "apps/leptos/src/pages/study.rs"
DASHBOARD_PAGE = ROOT / "apps/leptos/src/pages/dashboard.rs"
CREDENTIALS_PAGE = ROOT / "apps/leptos/src/pages/credentials.rs"
HOLOCHAIN_UI = ROOT / "apps/leptos/src/holochain.rs"
LEARNING_DOC = ROOT / "docs/learning-contract.md"
LIFECYCLE_DOC = ROOT / "docs/live-data-lifecycle.md"
LEPTOS_SETUP = ROOT / "apps/leptos/SETUP.md"
HAPP_MANIFEST = ROOT / "happ/happ.yaml"
DNA_MANIFEST = ROOT / "dna/dna.yaml"

CONSTANT_RE = re.compile(
    r'^pub const ([A-Z0-9_]+):\s*&str\s*=\s*"([^"]+)";', re.MULTILINE
)
EXTERN_RE = re.compile(r"#\[hdk_extern\]\s*pub fn\s+([a-zA-Z0-9_]+)")


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def manifest_names(source: str, indent: int) -> set[str]:
    prefix = " " * indent
    pattern = re.compile(rf'^{re.escape(prefix)}- name:\s*"?([^"\s]+)"?\s*$', re.MULTILINE)
    return set(pattern.findall(source))


def main() -> int:
    errors: list[str] = []
    contracts = read(CONTRACTS)
    coordinator = read(COORDINATOR)
    constants = dict(CONSTANT_RE.findall(contracts))

    required_constants = {
        "PRAXIS_ROLE_NAME",
        "LEARNING_COORDINATOR_ZOME",
        "GET_LEARNING_CONTRACT_FN",
        "LIST_COURSE_SUMMARIES_FN",
        "SYNC_PROGRESS_FN",
        "GET_MY_PROGRESS_FN",
        "INTEGRATION_COORDINATOR_ZOME",
        "GET_DASHBOARD_SNAPSHOT_FN",
        "CREDENTIAL_COORDINATOR_ZOME",
        "LIST_MY_CREDENTIAL_SUMMARIES_FN",
    }
    missing_constants = sorted(required_constants.difference(constants))
    if missing_constants:
        errors.append(f"missing shared constants: {', '.join(missing_constants)}")

    role_names = manifest_names(read(HAPP_MANIFEST), indent=2)
    role = constants.get("PRAXIS_ROLE_NAME")
    if role and role not in role_names:
        errors.append(f"shared role {role!r} is absent from happ/happ.yaml")

    coordinator_section = read(DNA_MANIFEST).split("\ncoordinator:\n", maxsplit=1)
    if len(coordinator_section) != 2:
        errors.append("dna/dna.yaml has no coordinator section")
        coordinator_names: set[str] = set()
    else:
        coordinator_names = manifest_names(coordinator_section[1], indent=4)
    zome = constants.get("LEARNING_COORDINATOR_ZOME")
    if zome and zome not in coordinator_names:
        errors.append(f"shared zome {zome!r} is absent from dna/dna.yaml")

    dashboard_zome = constants.get("INTEGRATION_COORDINATOR_ZOME")
    if dashboard_zome and dashboard_zome not in coordinator_names:
        errors.append(f"shared dashboard zome {dashboard_zome!r} is absent from dna/dna.yaml")

    credential_zome = constants.get("CREDENTIAL_COORDINATOR_ZOME")
    if credential_zome and credential_zome not in coordinator_names:
        errors.append(f"shared credential zome {credential_zome!r} is absent from dna/dna.yaml")

    externs = set(EXTERN_RE.findall(coordinator))
    for constant_name in (
        "GET_LEARNING_CONTRACT_FN",
        "LIST_COURSE_SUMMARIES_FN",
        "SYNC_PROGRESS_FN",
        "GET_MY_PROGRESS_FN",
    ):
        endpoint = constants.get(constant_name)
        if endpoint and endpoint not in externs:
            errors.append(
                f"{constant_name} names {endpoint!r}, but the learning coordinator "
                "does not export it"
            )

    integration_externs = set(EXTERN_RE.findall(read(INTEGRATION_COORDINATOR)))
    dashboard_endpoint = constants.get("GET_DASHBOARD_SNAPSHOT_FN")
    if dashboard_endpoint and dashboard_endpoint not in integration_externs:
        errors.append(
            f"GET_DASHBOARD_SNAPSHOT_FN names {dashboard_endpoint!r}, but the "
            "integration coordinator does not export it"
        )

    credential_source = read(CREDENTIAL_COORDINATOR)
    credential_externs = set(EXTERN_RE.findall(credential_source))
    credential_endpoint = constants.get("LIST_MY_CREDENTIAL_SUMMARIES_FN")
    if credential_endpoint and credential_endpoint not in credential_externs:
        errors.append(
            f"LIST_MY_CREDENTIAL_SUMMARIES_FN names {credential_endpoint!r}, but the "
            "credential coordinator does not export it"
        )

    usage_requirements = {
        COURSES_PAGE: {
            "LEARNING_COORDINATOR_ZOME",
            "LIST_COURSE_SUMMARIES_FN",
            "CourseSummary",
        },
        STUDY_PAGE: {
            "LEARNING_COORDINATOR_ZOME",
            "SYNC_PROGRESS_FN",
            "GET_MY_PROGRESS_FN",
            "ProgressSyncInput",
            "ProgressSyncReceipt",
            "ProgressSnapshot",
        },
        DASHBOARD_PAGE: {
            "INTEGRATION_COORDINATOR_ZOME",
            "GET_DASHBOARD_SNAPSHOT_FN",
            "DashboardSnapshot",
            "DASHBOARD_CONTRACT_VERSION",
        },
        CREDENTIALS_PAGE: {
            "CREDENTIAL_COORDINATOR_ZOME",
            "LIST_MY_CREDENTIAL_SUMMARIES_FN",
            "CredentialList",
            "CredentialSummary",
            "CREDENTIAL_CONTRACT_VERSION",
        },
    }
    for path, names in usage_requirements.items():
        source = read(path)
        for name in sorted(names):
            if name not in source:
                errors.append(f"{path.relative_to(ROOT)} does not use shared {name}")

    for dto in ("CourseSummary", "ProgressSyncInput", "ProgressSyncReceipt", "ProgressSnapshot"):
        if not re.search(rf"pub struct\s+{dto}\b", contracts):
            errors.append(f"shared DTO {dto} is not declared in praxis-core")
        if dto not in coordinator:
            errors.append(f"learning coordinator does not use shared DTO {dto}")

    dashboard_dtos = (
        "DashboardSnapshot",
        "DashboardGamification",
        "DashboardSkillMastery",
        "DashboardRecommendation",
        "DashboardActivity",
    )
    integration_source = read(INTEGRATION_COORDINATOR)
    dashboard_source = read(DASHBOARD_PAGE)
    for dto in dashboard_dtos:
        if not re.search(rf"pub struct\s+{dto}\b", contracts):
            errors.append(f"shared dashboard DTO {dto} is not declared in praxis-core")
        if dto not in integration_source:
            errors.append(f"integration coordinator does not use shared dashboard DTO {dto}")

    adaptive_externs = set(EXTERN_RE.findall(read(ADAPTIVE_COORDINATOR)))
    if "get_active_recommendations" not in adaptive_externs:
        errors.append("adaptive coordinator lacks the read-only recommendation endpoint")
    if "generate_recommendations" in integration_source[
        integration_source.find("pub fn get_dashboard_snapshot") :
    ]:
        errors.append("dashboard snapshot generates recommendations as a read side effect")

    retired_dashboard_endpoints = {
        "get_learner_stats",
        "get_streak",
        "get_due_summary",
        "get_top_skills",
        "get_recommendations",
        "get_recent_activity",
    }
    for endpoint in sorted(retired_dashboard_endpoints):
        if endpoint in dashboard_source:
            errors.append(f"dashboard still references nonexistent endpoint {endpoint!r}")

    for phrase in (
        "mode.get() != AppMode::Demo",
        "Demo-only concept simulation",
        "Values and statuses are fictional examples",
        "Illustrative control only",
    ):
        if phrase not in dashboard_source:
            errors.append(f"dashboard does not gate or disclose conceptual UI with {phrase!r}")

    for dto in ("CredentialSummary", "CredentialList"):
        if not re.search(rf"pub struct\s+{dto}\b", contracts):
            errors.append(f"shared credential DTO {dto} is not declared in praxis-core")
        if dto not in credential_source:
            errors.append(f"credential coordinator does not use shared DTO {dto}")

    credential_page_source = read(CREDENTIALS_PAGE)
    for phrase in (
        "LinkTypes::LearnerToCredentials",
        "credential.subject_id == learner_id",
        "is_valid: false",
        "signature and revocation checks are not implemented",
    ):
        if phrase not in credential_source:
            errors.append(f"credential coordinator does not enforce {phrase!r}")
    for retired_claim in (
        "set_verified.set(Some(true))",
        "Verified on Holochain",
        "Signature valid. Not revoked",
        '"get_my_credentials"',
        "CLR 2.0 Compliant",
        "Generated: March 20, 2026",
        "MATL Trust",
    ):
        if retired_claim in credential_page_source:
            errors.append(f"credential UI still contains false or retired claim {retired_claim!r}")
    for disclosure in (
        "Proof attached — verification not run",
        "Not verified in this build",
        "No revocation status is published",
        "Disclosure draft only",
        "Public portfolio listing is not connected",
    ):
        if disclosure not in credential_page_source:
            errors.append(f"credential UI does not disclose {disclosure!r}")

    readiness_requirements = {
        HOLOCHAIN_UI: {
            "DataSource",
            "ResourceState",
            "tracked_data_source",
            "zome_call_signing_ready",
            "Live signer unavailable",
            "refresh_zome_call_signing_ready",
        },
        STUDY_PAGE: {"zome_call_signing_ready", "refresh_zome_call_signing_ready"},
        LEPTOS_SETUP: {
            "__HC_LAUNCHER_ENV__",
            "__HC_ZOME_CALL_SIGNER__",
            "Uint8Array",
        },
        LIFECYCLE_DOC: {
            "WaitingForLive",
            "LiveError",
            "Ready(empty)",
            "zome_calls_ready_untracked",
            "must not mount nested providers",
        },
    }
    for path, phrases in readiness_requirements.items():
        source = read(path)
        for phrase in sorted(phrases):
            if phrase not in source:
                errors.append(
                    f"{path.relative_to(ROOT)} does not disclose or enforce {phrase!r}"
                )

    live_resource_pages = {
        COURSES_PAGE: 1,
        CREDENTIALS_PAGE: 1,
        DASHBOARD_PAGE: 1,
        ROOT / "apps/leptos/src/pages/profile.rs": 2,
    }
    for path, minimum in live_resource_pages.items():
        source = read(path)
        tracked_count = source.count("tracked_data_source(")
        if tracked_count < minimum:
            errors.append(
                f"{path.relative_to(ROOT)} tracks only {tracked_count} Live resources; "
                f"expected at least {minimum}"
            )
        for state in ("WaitingForLive", "LiveError"):
            if state not in source:
                errors.append(
                    f"{path.relative_to(ROOT)} does not distinguish {state}"
                )

    profile_source = read(ROOT / "apps/leptos/src/pages/profile.rs")
    if "<HolochainProvider" in profile_source:
        errors.append("profile page mounts a nested HolochainProvider")
    if ".is_mock()" in profile_source:
        errors.append("profile mutations still infer readiness from mock status")
    if profile_source.count("zome_calls_ready_untracked()") < 3:
        errors.append("profile mutations do not all enforce Live signing readiness")

    if re.search(r"zero(?:ed)? signature", read(LEARNING_DOC), re.IGNORECASE):
        errors.append("learning contract still describes zero-signature zome calls")

    if errors:
        print("Learning contract validation failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    endpoint_count = sum(
        constants[name] in externs
        for name in (
            "GET_LEARNING_CONTRACT_FN",
            "LIST_COURSE_SUMMARIES_FN",
            "SYNC_PROGRESS_FN",
            "GET_MY_PROGRESS_FN",
        )
    )
    print(
        "Learning contract valid: "
        f"role={role}, learning_zome={zome}, dashboard_zome={dashboard_zome}, "
        f"endpoints={endpoint_count + int(dashboard_endpoint in integration_externs) + int(credential_endpoint in credential_externs)}, "
        f"shared_dtos={6 + len(dashboard_dtos)}, "
        "live_signing_gate=present, reactive_live_resources=present."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
