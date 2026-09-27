#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

EXPECTED_CLASSES = [
    "DesignPack",
    "DeploymentPack",
    "ConformancePack",
    "OperationsPack",
    "TrainingPack",
    "PhysicalStarterPack",
    "FederationIntroductionPack",
]
EXPECTED_IDENTITIES = {"NodeIdentity", "HostIdentity", "ApplicationAgentIdentity"}
REQUIRED_FORBIDDEN = {
    "source-node-private-keys",
    "source-host-ssh-identity",
    "source-application-agent-keys",
    "source-xenia-signing-secrets",
    "source-governance-roles",
    "source-authority-receipts",
    "source-reputation",
    "source-standing",
    "source-itc-or-account-balances",
    "resident-or-person-secrets",
    "reusable-default-bootstrap-credentials",
}
REQUIRED_EVIDENCE = {
    "seed-package-commitment",
    "component-admission-records",
    "fresh-identity-generation-events-without-secret-disclosure",
    "local-governance-initialization-record",
    "adaptation-and-loss-records",
    "remaining-external-dependency-map",
    "seeder-unavailability-transition",
    "local-operation-observations-during-seeder-outage",
    "optional-later-federation-event",
    "second-generation-seed-package-export",
}
ALLOWED_DECISIONS = {
    "AcceptAsIs",
    "AcceptWithLocalAdaptation",
    "Reject",
    "Defer",
    "MoreEvidenceRequired",
}


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def validate(package, run):
    errors = []

    def require(condition, message):
        if not condition:
            errors.append(message)

    require(package.get("profile_id") == "myc-int-007k-node-seed-package-v1",
            "unexpected package profile_id")
    require(run.get("profile_id") == "myc-int-007k-bootstrap-run-v1",
            "unexpected bootstrap-run profile_id")
    require(package.get("profile_version") == "1.0.0" and run.get("profile_version") == "1.0.0",
            "profile version drift")
    require(package.get("status") == "design-fixture" and run.get("status") == "design-fixture",
            "fixture status drift")
    require(package.get("mode") == "FreshNode" and run.get("mode") == "FreshNode",
            "007K is FreshNode-only")
    require(
        package.get("source_node") == "A"
        and package.get("intended_recipient") == "B"
        and run.get("receiving_node") == "B",
        "A->B source/recipient binding drift",
    )
    require(package.get("authority") == "None", "seed package acquired authority")
    require(package.get("secret_free_required") is True, "secret-free requirement removed")
    require(package.get("fresh_identity_required") is True, "fresh-identity requirement removed")
    require(package.get("federation_required") is False, "package now requires federation")

    components = package.get("components", [])
    require([item.get("class") for item in components] == EXPECTED_CLASSES,
            "seed-package class set/order drift")
    component_ids = [item.get("component_id") for item in components]
    require(len(component_ids) == 7 and len(set(component_ids)) == 7,
            "seed-package component IDs must be seven unique values")

    for component in components:
        cid = component.get("component_id", "<unknown>")
        require(component.get("authority") == "None", f"{cid}: authority must remain None")
        require(component.get("admission_optional") is True, f"{cid}: selective admission removed")
        require(bool(component.get("source_generation")), f"{cid}: missing source generation")

    require(REQUIRED_FORBIDDEN <= set(package.get("forbidden_inheritance", [])),
            "required forbidden-inheritance protection missing")
    require(all(value is False for value in package.get("claim_ceiling", {}).values()),
            "package claim ceiling upgraded")

    require(run.get("seed_package_generation") == package.get("package_generation"),
            "package/run generation mismatch")
    require(run.get("bootstrap_state") == "Planned",
            "design fixture must not claim executed bootstrap")

    admissions = run.get("component_admission", [])
    admission_ids = [item.get("component_id") for item in admissions]
    require(admission_ids == component_ids,
            "bootstrap admission set/order must exactly cover package components")

    for admission in admissions:
        cid = admission.get("component_id", "<unknown>")
        decision = admission.get("decision")
        require(decision in ALLOWED_DECISIONS, f"{cid}: unsupported admission decision")
        if decision in {"AcceptAsIs", "AcceptWithLocalAdaptation"}:
            require(bool(admission.get("local_generation")),
                    f"{cid}: accepted/adapted component needs receiving-node generation")
        else:
            require(admission.get("local_generation") is None,
                    f"{cid}: non-admitted component cannot have local generation")

    identities = run.get("identity_events_required", [])
    require({item.get("identity") for item in identities} == EXPECTED_IDENTITIES,
            "fresh identity set drift")
    for identity in identities:
        name = identity.get("identity", "<unknown>")
        require(identity.get("fresh") is True, f"{name}: identity must be fresh")
        require(identity.get("copied_from_seeder") is False,
                f"{name}: identity copied from seeder")

    governance = run.get("local_governance", {})
    require(governance.get("owner") == "ReceivingNodeB"
            and governance.get("initialized_locally") is True,
            "receiving node no longer owns local governance")
    require(governance.get("source_node_roles_inherited") is False,
            "source governance roles inherited")
    require(governance.get("source_node_standing_inherited") is False,
            "source standing inherited")

    require(all(value is False for value in run.get("foreign_artifact_rules", {}).values()),
            "foreign artifact auto-promotion enabled")

    federation = run.get("federation", {})
    require(federation.get("required_for_bootstrap") is False,
            "federation became bootstrap dependency")
    require(federation.get("joined_during_bootstrap") is False,
            "bootstrap silently joined federation")
    require(federation.get("may_join_later") is True
            and federation.get("may_leave_later") is True,
            "post-bootstrap federation choice narrowed")

    independence = run.get("independence_test", {})
    require(independence.get("seeder_A_available_initially") is True,
            "initial seeder availability assumption drift")
    require(independence.get("seeder_A_removed_after_package_transfer") is True,
            "seeder outage test removed")
    require(independence.get("required_local_operation_interval_seconds") == 86400,
            "007K v1 independence interval must remain exactly 86,400 seconds")
    require(independence.get("external_dependencies_must_remain_explicit") is True,
            "external dependency disclosure removed")
    require(bool(independence.get("required_capabilities_under_test")),
            "independence capability set became empty")

    recursive = run.get("recursive_seeding_requirement", {})
    require(recursive.get("B_must_later_produce_own_seed_package") is True,
            "B no longer required to emit second-generation package")
    require(recursive.get("A_required_for_B_to_C") is False,
            "A became required for B->C")
    require(recursive.get("B_to_C_uses_same_contract") is True,
            "B->C no longer uses same bootstrap contract")
    require(recursive.get("lineage_provenance_retained") is True,
            "public lineage provenance removed")
    require(recursive.get("authority_lineage_inherited") is False,
            "authority lineage became inheritable")

    require(REQUIRED_EVIDENCE <= set(run.get("evidence_required", [])),
            "required bootstrap/independence evidence missing")
    require(all(value is False for value in run.get("claim_ceiling", {}).values()),
            "bootstrap run claim ceiling upgraded")

    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("package")
    parser.add_argument("run")
    args = parser.parse_args()

    errors = validate(load_json(args.package), load_json(args.run))
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        raise SystemExit(1)

    print("MYC-INT-007K seed/bootstrap fixtures: PASS")


if __name__ == "__main__":
    main()
