#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

PROFILE_ID = "mycelix-governance-source-intent-20452d0c-v1"
PROFILE_SHA256 = "557282ebfdf98c14194b34db3412a6eadd97fabab06073cc5772b87101561003"
SUBJECT_SHA = "20452d0cde9448c424d708d6307aa33a3d1901e3"
SUBJECT_TREE_SHA = "4d24feadca2f076fac8e781f16350ce313f88995"
AUTHORITY = "SourceIntentBound"
CEILING = "SourceIntentOnly"

INTEGRITY_NAMES = [
    "proposals_integrity", "voting_integrity", "execution_integrity",
    "constitution_integrity", "governance_bridge_integrity",
    "threshold_signing_integrity", "councils_integrity",
    "jurisdiction_integrity", "budgeting_integrity",
]
COORDINATOR_NAMES = [
    "proposals", "voting", "execution", "constitution", "governance_bridge",
    "threshold_signing", "councils", "jurisdiction", "budgeting",
]

def canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode()

def load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("profile root must be object")
    return value

def payload_digest(profile: dict) -> str:
    payload = dict(profile)
    payload.pop("profile_content_sha256", None)
    return hashlib.sha256(canonical(payload)).hexdigest()

def eq(actual: object, expected: object, name: str) -> None:
    if actual != expected:
        raise ValueError(f"{name} drift: {actual!r}")

def validate(profile: dict) -> dict:
    eq(profile.get("schema"), "mycelix-holochain-governance-source-intent-profile-v1", "schema")
    eq(profile.get("profile_id"), PROFILE_ID, "profile id")
    eq(profile.get("profile_revision"), 1, "revision")
    eq(profile.get("authority_class"), AUTHORITY, "authority")
    eq(profile.get("claim_ceiling"), CEILING, "claim ceiling")
    eq(profile.get("profile_content_sha256"), PROFILE_SHA256, "commitment field")
    eq(payload_digest(profile), PROFILE_SHA256, "payload commitment")

    source = profile["source_binding"]
    eq(source["repository"], "Luminous-Dynamics/mycelix", "repository")
    eq(source["subject_sha"], SUBJECT_SHA, "subject")
    eq(source["subject_tree_sha"], SUBJECT_TREE_SHA, "subject tree")
    eq(source["dna_manifest"], {
        "path": "mycelix-governance/dna/dna.yaml",
        "git_blob_sha1": "972a9f32d43ce645aa71900eda11d1756d2e7055",
    }, "DNA manifest")
    eq(source["cargo_manifest"], {
        "path": "mycelix-governance/Cargo.toml",
        "git_blob_sha1": "219cbd48e02692c75a49f0b04a4130bf897aaa5c",
    }, "Cargo manifest")
    eq(source["cargo_lock"], {
        "path": "mycelix-governance/Cargo.lock",
        "git_blob_sha1": "f872b74f13264e7345be02b64270179eed56c703",
    }, "Cargo lock")
    eq(source["rust_toolchain_pin_observation"], "NoRustToolchainFileInExactSubjectTree", "toolchain observation")

    holo = profile["declared_holochain_profile"]
    eq(holo, {
        "hdk": "=0.6.1",
        "hdi": "=0.7.1",
        "holochain_integrity_types": "=0.6.1",
        "holochain_zome_types": "=0.6.1",
        "holo_hash": "=0.6.1",
        "hdk_derive": "=0.6.1",
        "holochain_serialized_bytes": "=0.0.57",
        "runtime_family_claim": "Holochain0.6FamilyByDependencyDeclarationOnly",
    }, "declared Holochain profile")

    dna = profile["dna_intent"]
    eq(dna["manifest_version"], "0", "manifest version")
    eq(dna["name"], "mycelix_governance_dna", "DNA name")
    eq(dna["network_seed"], "mycelix-governance-v1", "network seed")
    eq([z["name"] for z in dna["integrity_zomes"]], INTEGRITY_NAMES, "integrity zomes")
    eq([z["name"] for z in dna["coordinator_zomes"]], COORDINATOR_NAMES, "coordinator zomes")
    eq(len(dna["integrity_zomes"]), 9, "integrity count")
    eq(len(dna["coordinator_zomes"]), 9, "coordinator count")

    positive = [
        "ExactSourceSubjectBound", "ExactDnaManifestBlobBound",
        "ExactCargoManifestBlobBound", "ExactCargoLockBlobBound",
        "NineIntegrityZomesDeclared", "NineCoordinatorZomesDeclared",
        "NetworkSeedDeclared", "HolochainDependencyVersionsDeclared",
    ]
    eq(profile["positive_observations"], positive, "positive observations")

    unqualified = [
        "RustToolchainPinQualified", "BuiltWasmIdentityQualified",
        "BuiltDnaBundleIdentityQualified", "DnaHashQualified",
        "HappBundleIdentityQualified", "InstalledAppCellIdentityQualified",
        "RuntimeProcessAssociationQualified", "FreshRuntimeWitnessQualified",
        "ContinuousRuntimeCurrentnessQualified",
        "AuthorizedDeploymentCurrentnessQualified", "DeploymentSafetyQualified",
    ]
    eq(profile["unsupported_or_unqualified"], unqualified, "unqualified properties")

    return {
        "profile_id": PROFILE_ID,
        "profile_content_sha256": PROFILE_SHA256,
        "authority_class": AUTHORITY,
        "claim_ceiling": CEILING,
        "subject_sha": SUBJECT_SHA,
        "subject_tree_sha": SUBJECT_TREE_SHA,
        "integrity_zome_count": 9,
        "coordinator_zome_count": 9,
        "positive_observation_count": len(positive),
        "unqualified_property_count": len(unqualified),
        "deployment_currentness_qualified": False,
        "valid": True,
    }

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(validate(load(args.profile)), sort_keys=True, separators=(",", ":")))

if __name__ == "__main__":
    main()
