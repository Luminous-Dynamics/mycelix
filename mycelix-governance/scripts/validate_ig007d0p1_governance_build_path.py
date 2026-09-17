#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

PROFILE_ID = "mycelix-governance-source-intent-ee3582af-v2"
PROFILE_SHA256 = "d2cd07e94274ba9ce8395ab5213db656cd33aa022005de065f8ca3715c0099bd"
PRODUCT_SHA = "ee3582afdf7866d8cf1096cf4996e127d35a8a5e"
PREDECESSOR_SHA = "557282ebfdf98c14194b34db3412a6eadd97fabab06073cc5772b87101561003"


def canonical(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")


def eq(actual: object, expected: object, label: str) -> None:
    if actual != expected:
        raise ValueError(f"{label} drift: {actual!r}")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--profile", type=Path, required=True)
    args = p.parse_args()
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    if not isinstance(profile, dict):
        raise ValueError("profile root must be object")

    payload = dict(profile)
    payload.pop("profile_content_sha256", None)
    digest = hashlib.sha256(canonical(payload)).hexdigest()

    eq(profile.get("schema"), "mycelix-governance-source-intent-successor-profile-v1", "schema")
    eq(profile.get("profile_id"), PROFILE_ID, "profile id")
    eq(profile.get("profile_revision"), 2, "revision")
    eq(profile.get("authority_class"), "SourceIntentSuccessorBound", "authority")
    eq(profile.get("claim_ceiling"), "RepositoryContainedGovernanceBuildPathBound", "claim ceiling")
    eq(profile.get("profile_content_sha256"), PROFILE_SHA256, "commitment field")
    eq(digest, PROFILE_SHA256, "payload commitment")

    predecessor = profile.get("predecessor", {})
    eq(predecessor.get("content_sha256"), PREDECESSOR_SHA, "predecessor commitment")
    eq(predecessor.get("historical_source_subject_sha"), "20452d0cde9448c424d708d6307aa33a3d1901e3", "historical source")
    eq(predecessor.get("historical_evidence_head_sha"), "fe3e9097ed19ff1ee29fd49b908e2b5086f06dd1", "historical evidence head")
    eq(predecessor.get("qualified_by_run"), 35154741664, "predecessor qualification run")

    source = profile.get("source_binding", {})
    eq(source.get("repository"), "Luminous-Dynamics/mycelix", "repository")
    eq(source.get("repaired_product_subject_sha"), PRODUCT_SHA, "product subject")
    eq(source.get("base_source_subject_sha"), "20452d0cde9448c424d708d6307aa33a3d1901e3", "base subject")
    eq(source.get("delta_class"), "OneFlakeImportPathReplacement", "delta class")
    eq(source.get("flake"), {"path":"mycelix-governance/flake.nix","git_blob_sha1":"70006c71f02a560bd873526dd8b882cbba9e639f"}, "flake binding")
    eq(source.get("flake_lock"), {"path":"mycelix-governance/flake.lock","git_blob_sha1":"faae625ed2fb9a7ecac236a99e342f7bdc94a644"}, "flake lock binding")
    eq(source.get("shared_holochain_module"), {"path":"nix/modules/holochain-base.nix","git_blob_sha1":"e9015df8f82520d8c3607026de126216412727b6"}, "module binding")
    eq(source.get("cargo_lock"), {"path":"mycelix-governance/Cargo.lock","git_blob_sha1":"f872b74f13264e7345be02b64270179eed56c703"}, "Cargo lock binding")
    eq(source.get("dna_manifest"), {"path":"mycelix-governance/dna/dna.yaml","git_blob_sha1":"972a9f32d43ce645aa71900eda11d1756d2e7055"}, "DNA binding")

    semantics = profile.get("path_semantics", {})
    eq(semantics.get("historical_import"), "../../nix/modules/holochain-base.nix", "historical import")
    eq(semantics.get("historical_disposition"), "EscapesRepositoryRoot", "historical disposition")
    eq(semantics.get("successor_import"), "../nix/modules/holochain-base.nix", "successor import")
    eq(semantics.get("successor_disposition"), "RepositoryContained", "successor disposition")
    eq(semantics.get("normalized_successor_target"), "nix/modules/holochain-base.nix", "normalized target")

    unqualified = profile.get("explicitly_unqualified", [])
    if len(unqualified) != 12:
        raise ValueError(f"unexpected unqualified property count: {len(unqualified)}")

    result = {
        "valid": True,
        "profile_id": PROFILE_ID,
        "profile_content_sha256": PROFILE_SHA256,
        "authority_class": profile["authority_class"],
        "claim_ceiling": profile["claim_ceiling"],
        "repaired_product_subject_sha": PRODUCT_SHA,
        "predecessor_profile_content_sha256": PREDECESSOR_SHA,
        "repository_contained": True,
        "unqualified_property_count": len(unqualified),
    }
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
