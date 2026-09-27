#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

EXPECTED_PARENT = "284a96e20102f5beeec5533c80839b5aa2c3c65c"
EXPECTED_GENERATIONS = [f"N{i}" for i in range(8)]
EXPECTED_SEED_CLASSES = [
    "DesignPack",
    "DeploymentPack",
    "ConformancePack",
    "OperationsPack",
    "TrainingPack",
    "PhysicalStarterPack",
    "FederationIntroductionPack",
]
EXPECTED_ADVERSARIAL = [f"S{i:02d}" for i in range(1, 19)]
REQUIRED_CAPABILITIES = {
    "food", "water", "energy", "repair-fabrication", "compute-network",
    "governance-operations", "evidence-provenance", "health-safety-support",
    "external-legal-finance", "critical-imports", "skills-maintainers",
    "recovery-spares", "federation-dependency",
}
REQUIRED_IMPORT_RULES = {
    "receiving-node-selects-each-component",
    "receiving-node-generates-own-identity-and-secrets",
    "every-imported-artifact-keeps-source-generation",
    "local-policy-may-reject-any-component",
    "local-adaptation-creates-new-generation",
    "declared-translation-loss-required-where-applicable",
    "foreign-certification-requires-explicit-local-recognition",
    "training-does-not-create-standing",
    "donation-does-not-create-authority",
    "federation-membership-is-optional",
}
REQUIRED_CORE_INVARIANTS = {
    "capability-growth-does-not-create-authority-over-other-nodes",
    "seed-support-does-not-create-governance-standing",
    "resource-transfer-does-not-create-authority",
    "deployment-artifact-does-not-copy-node-identity",
    "foreign-certification-does-not-become-local-certification-automatically",
    "reputation-credit-or-standing-do-not-copy-across-bootstrap",
    "new-node-can-operate-before-federating",
    "new-node-can-decline-or-leave-federation",
    "remaining-critical-imports-remain-visible",
    "self-sufficiency-is-not-a-binary-field",
}
FORBIDDEN_KEYS = {
    "self_sustaining", "self-sustaining", "maturity_score", "overall_score",
    "self_sufficiency_score", "self-sufficiency-score",
}


class ValidationError(ValueError):
    pass


def _require(condition, message):
    if not condition:
        raise ValidationError(message)


def _walk(value):
    if isinstance(value, dict):
        yield value
        for child in value.values():
            yield from _walk(child)
    elif isinstance(value, list):
        for child in value:
            yield from _walk(child)


def validate_profile(doc):
    _require(isinstance(doc, dict), "root must be an object")
    _require(doc.get("profile_id") == "myc-int-007i-node-maturation-seeding-v1",
             "unexpected profile_id")
    _require(doc.get("profile_version") == "1.0.0", "unexpected profile_version")
    _require(doc.get("parent_subject") == EXPECTED_PARENT, "parent subject drift")

    for obj in _walk(doc):
        if isinstance(obj, dict):
            forbidden = FORBIDDEN_KEYS.intersection(obj)
            _require(not forbidden, f"forbidden scalar/binary maturity key(s): {sorted(forbidden)}")

    claims = doc.get("claim_ceiling")
    _require(isinstance(claims, dict) and claims, "missing claim_ceiling")
    _require(all(v is False for v in claims.values()), "all claim ceilings must remain false")

    core = set(doc.get("core_invariants", []))
    _require(REQUIRED_CORE_INVARIANTS <= core, "missing core independence invariant")

    dims = doc.get("capability_dimensions")
    _require(isinstance(dims, list), "capability_dimensions must be a list")
    dim_ids = [d.get("id") for d in dims]
    _require(len(dim_ids) == len(set(dim_ids)), "duplicate capability dimension")
    _require(REQUIRED_CAPABILITIES <= set(dim_ids), "required capability dimension missing")
    _require(all(d.get("scalar_score") is False for d in dims),
             "capability dimensions must remain unscored")

    gens = doc.get("lifecycle_generations")
    _require(isinstance(gens, list), "lifecycle_generations must be a list")
    _require([g.get("id") for g in gens] == EXPECTED_GENERATIONS,
             "lifecycle generation set/order must remain N0..N7")
    by_gen = {g["id"]: g for g in gens}

    n5 = set(by_gen["N5"].get("required_properties", []))
    _require("seed-package-contains-no-source-node-private-keys" in n5,
             "N5 must prohibit source private-key inheritance")
    _require("seed-package-grants-no-governance-authority" in n5,
             "N5 seed package must grant no governance authority")
    _require("receiving-node-can-selectively-admit-components" in n5,
             "N5 must preserve selective admission")

    n6 = set(by_gen["N6"].get("required_properties", []))
    for required in {
        "new-node-identity-and-keys",
        "local-governance-owned-by-receiving-community",
        "local-certification-not-implicitly-inherited",
        "can-operate-before-federation",
        "can-decline-or-leave-federation",
    }:
        _require(required in n6, f"N6 missing required property: {required}")

    n7 = set(by_gen["N7"].get("required_properties", []))
    _require("seeded-node-can-create-own-seed-package" in n7,
             "N7 must require second-generation seed package creation")
    _require("original-seeder-not-required" in n7,
             "N7 must not require original seeder")

    seed_classes = doc.get("seed_package_classes")
    _require(isinstance(seed_classes, list), "seed_package_classes must be a list")
    _require([s.get("id") for s in seed_classes] == EXPECTED_SEED_CLASSES,
             "seed-package class set/order drift")
    _require(all(s.get("authority") == "None" for s in seed_classes),
             "every seed-package class must have authority=None")

    import_rules = set(doc.get("seed_import_rules", []))
    _require(REQUIRED_IMPORT_RULES <= import_rules, "seed import independence rule missing")

    cases = doc.get("adversarial_cases")
    _require(isinstance(cases, list), "adversarial_cases must be a list")
    _require([c.get("id") for c in cases] == EXPECTED_ADVERSARIAL,
             "adversarial case set/order must remain S01..S18")

    mapping = doc.get("showcase_mapping", {})
    _require(mapping.get("node_A_to_B_relationship") == "SeederPeerNotParentAuthority",
             "Node A must remain seeder peer, not parent authority")
    _require(mapping.get("node_B_future_role") ==
             "first independently bootstrapped physical or semi-physical second node",
             "Node B future role must remain independently bootstrapped")

    optional = doc.get("runtime_optionalities", {})
    _require(optional.get("federation_required_for_local_operation") is False,
             "federation must remain optional for local operation")
    _require(optional.get("seeder_online_required_after_complete_bootstrap") is False,
             "seeder must not remain permanently required")

    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", type=Path)
    args = parser.parse_args()
    with args.profile.open("r", encoding="utf-8") as f:
        doc = json.load(f)
    validate_profile(doc)
    print("MYC-INT-007I profile validation: PASS")


if __name__ == "__main__":
    main()
