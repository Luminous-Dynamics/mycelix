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
DERIVED_CLASSES = {
    "DesignPack",
    "DeploymentPack",
    "ConformancePack",
    "OperationsPack",
    "TrainingPack",
}
LOCAL_B_CLASSES = {"PhysicalStarterPack", "FederationIntroductionPack"}
EXPECTED_IDENTITIES = {"NodeIdentity", "HostIdentity", "ApplicationAgentIdentity"}
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


def validate(a_package, b_run, b_package, c_run):
    errors = []

    def require(condition, message):
        if not condition:
            errors.append(message)

    require(
        b_package.get("contract_profile") == a_package.get("profile_id"),
        "B package no longer reuses 007K package contract",
    )
    require(
        c_run.get("contract_profile") == b_run.get("profile_id"),
        "C run no longer reuses 007K bootstrap contract",
    )
    require(
        b_package.get("source_node") == "B"
        and b_package.get("intended_recipient") == "C"
        and c_run.get("receiving_node") == "C",
        "B->C source/recipient binding drift",
    )
    require(
        b_package.get("mode") == "FreshNode" and c_run.get("mode") == "FreshNode",
        "recursive fixture is FreshNode-only",
    )
    require(b_package.get("package_generation") == "B-seed-0001",
            "second-generation package must be B-owned generation")
    require(b_package.get("authority") == "None", "B package acquired authority")
    require(
        b_package.get("original_seeder_A_required") is False
        and b_package.get("original_seeder_A_available") is False,
        "original seeder A became required/available for B package",
    )
    require(
        b_package.get("secret_free_required") is True
        and b_package.get("fresh_identity_required") is True,
        "B package secret/fresh-identity requirements drift",
    )
    require(b_package.get("federation_required") is False,
            "B package now requires federation")

    a_components = a_package.get("components", [])
    b_components = b_package.get("components", [])
    require([item.get("class") for item in a_components] == EXPECTED_CLASSES,
            "A package class baseline drift")
    require([item.get("class") for item in b_components] == EXPECTED_CLASSES,
            "B package class set/order drift")

    b_ids = [item.get("component_id") for item in b_components]
    require(len(b_ids) == 7 and len(set(b_ids)) == 7,
            "B package component IDs must be seven unique values")

    a_by_class = {item["class"]: item for item in a_components}
    a_id_by_class = {item["class"]: item["component_id"] for item in a_components}
    b_admission_by_id = {
        item["component_id"]: item for item in b_run.get("component_admission", [])
    }

    for component in b_components:
        cls = component.get("class")
        cid = component.get("component_id", "<unknown>")
        require(component.get("authority") == "None", f"{cid}: authority must remain None")
        require(component.get("admission_optional") is True,
                f"{cid}: selective admission removed")
        require(bool(component.get("source_generation")),
                f"{cid}: missing B source generation")

        if cls in DERIVED_CLASSES:
            source_a_id = a_id_by_class[cls]
            source_admission = b_admission_by_id.get(source_a_id, {})
            require(
                source_admission.get("decision") in {"AcceptAsIs", "AcceptWithLocalAdaptation"},
                f"{cls}: B did not admit source A artifact",
            )
            require(
                component.get("origin_kind")
                == "DerivedFromAcceptedOrAdaptedAArtifact",
                f"{cls}: derived origin kind drift",
            )
            require(
                component.get("source_admission_ref")
                == source_admission.get("local_generation"),
                f"{cls}: B source-admission reference mismatch",
            )
            require(
                component.get("source_generation")
                == source_admission.get("local_generation"),
                f"{cls}: B package source is not B-owned local generation",
            )
            require(
                a_by_class[cls].get("source_generation")
                in component.get("public_provenance_lineage", []),
                f"{cls}: A-origin public provenance stripped",
            )
        elif cls in LOCAL_B_CLASSES:
            require(component.get("origin_kind") == "LocallyCreatedAtB",
                    f"{cls}: local-B origin kind drift")
            require(component.get("source_admission_ref") is None,
                    f"{cls}: fabricated A->B admission reference")
            require(component.get("public_provenance_lineage") == [],
                    f"{cls}: fabricated A-origin provenance")

    prohibited = b_package.get("prohibited_runtime_dependencies", [])
    require(
        set(prohibited)
        >= {"A-online-service", "A-secret-service", "A-authority-service", "A-governance-service"},
        "A runtime-dependency prohibition weakened",
    )

    lineage = b_package.get("lineage_policy", {})
    require(lineage.get("public_provenance_may_reference_A_origin") is True,
            "public provenance lineage disabled")
    require(lineage.get("authority_lineage_inherited") is False,
            "authority lineage became inheritable")
    require(lineage.get("secret_lineage_inherited") is False,
            "secret lineage became inheritable")
    require(all(value is False for value in b_package.get("claim_ceiling", {}).values()),
            "B package claim ceiling upgraded")

    require(
        c_run.get("seed_package_generation") == b_package.get("package_generation"),
        "B package / C run generation mismatch",
    )
    require(c_run.get("bootstrap_state") == "Planned",
            "C design fixture must not claim executed bootstrap")

    admissions = c_run.get("component_admission", [])
    require([item.get("component_id") for item in admissions] == b_ids,
            "C admission set/order must exactly cover B package")

    for admission in admissions:
        cid = admission.get("component_id", "<unknown>")
        decision = admission.get("decision")
        require(decision in ALLOWED_DECISIONS, f"{cid}: unsupported C admission decision")
        if decision in {"AcceptAsIs", "AcceptWithLocalAdaptation"}:
            local_generation = admission.get("local_generation")
            require(
                isinstance(local_generation, str) and local_generation.startswith("C-"),
                f"{cid}: accepted/adapted component needs C-owned generation",
            )
        else:
            require(admission.get("local_generation") is None,
                    f"{cid}: non-admitted component cannot gain local generation")

    original_a = c_run.get("original_seeder_A", {})
    require(original_a.get("required") is False,
            "A became required during C bootstrap")
    require(original_a.get("available_during_package_production") is False,
            "A became available during B package production")
    require(original_a.get("available_during_bootstrap") is False,
            "A became available during C bootstrap")

    identities = c_run.get("identity_events_required", [])
    require({item.get("identity") for item in identities} == EXPECTED_IDENTITIES,
            "C identity set drift")
    for identity in identities:
        name = identity.get("identity", "<unknown>")
        require(identity.get("fresh") is True, f"{name}: C identity must be fresh")
        require(identity.get("copied_from_seeder_B") is False,
                f"{name}: C identity copied from B")
        require(identity.get("copied_from_original_A") is False,
                f"{name}: C identity copied from A")

    governance = c_run.get("local_governance", {})
    require(
        governance.get("owner") == "ReceivingNodeC"
        and governance.get("initialized_locally") is True,
        "C no longer owns local governance",
    )
    for key in (
        "B_roles_inherited",
        "B_standing_inherited",
        "A_roles_inherited",
        "A_standing_inherited",
    ):
        require(governance.get(key) is False, f"C governance inheritance enabled: {key}")

    require(all(value is False for value in c_run.get("foreign_artifact_rules", {}).values()),
            "A/B foreign artifact auto-promotion enabled")

    federation = c_run.get("federation", {})
    require(federation.get("required_for_bootstrap") is False,
            "C bootstrap now requires federation")
    require(federation.get("joined_during_bootstrap") is False,
            "C bootstrap silently joined federation")
    require(
        federation.get("may_join_later") is True
        and federation.get("may_leave_later") is True,
        "C post-bootstrap federation choice narrowed",
    )

    independence = c_run.get("independence_test", {})
    require(independence.get("seeder_B_available_initially") is True,
            "B initial package-transfer role drift")
    require(independence.get("seeder_B_removed_after_package_transfer") is True,
            "B outage test removed")
    require(independence.get("original_A_required") is False,
            "A dependency reintroduced in C independence test")
    require(independence.get("required_local_operation_interval_seconds") == 86400,
            "007M v1 C independence interval must remain exactly 86,400 seconds")
    require(independence.get("external_dependencies_must_remain_explicit") is True,
            "C external-dependency disclosure removed")

    recursive = c_run.get("recursive_seeding_requirement", {})
    require(recursive.get("C_may_later_produce_own_seed_package") is True,
            "C future seeding capability removed")
    require(recursive.get("B_required_for_C_to_D") is False,
            "B became required for C->D")
    require(recursive.get("A_required_for_C_to_D") is False,
            "A became required for C->D")
    require(recursive.get("C_to_D_uses_same_contract") is True,
            "C->D no longer uses the same contract")
    require(recursive.get("lineage_provenance_retained") is True,
            "public lineage provenance removed")
    require(recursive.get("authority_lineage_inherited") is False,
            "authority lineage became inheritable at C")

    require(all(value is False for value in c_run.get("claim_ceiling", {}).values()),
            "C run claim ceiling upgraded")

    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("a_package")
    parser.add_argument("b_run")
    parser.add_argument("b_package")
    parser.add_argument("c_run")
    args = parser.parse_args()

    errors = validate(
        load_json(args.a_package),
        load_json(args.b_run),
        load_json(args.b_package),
        load_json(args.c_run),
    )
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        raise SystemExit(1)

    print("MYC-INT-007M recursive seeding fixtures: PASS")


if __name__ == "__main__":
    main()
