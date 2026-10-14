#!/usr/bin/env python3
"""Validate the Economy Atlas discovery registry using only the Python standard library.

This validates registry structure and provenance link consistency. It does NOT execute
or qualify any economic model, prove source contents, or validate empirical conclusions.
"""
from __future__ import annotations

import argparse
import json
import re
from datetime import date
import sys
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = ROOT / "docs" / "economy-atlas" / "model-registry.v1.json"
SHA40 = re.compile(r"^[0-9a-f]{40}$")
EPISTEMIC_STATUSES = {
    "implemented_source",
    "paper_reported",
    "design_only",
    "conceptual",
    "discovery_required",
    "external_framework",
}
REQUIRED_VARIANTS = {
    "mycelix.finance.tricurrency.v1",
    "symthaea.economy.thermodynamic-ledger.v1",
    "symthaea.concepts.tricurrency.essay16.v1",
}
REQUIRED_REGENERATIVE_MODELS = {
    "regenerative.economics.energy-network-science.v1",
    "regenerative.doughnut.place-portrait.v1",
    "regenerative.circular-economy.v1",
    "regenerative.land.agroecology-outcomes.v1",
}


def validate_registry(data: object) -> list[str]:
    """Return all contract violations; an empty list means the registry checks pass."""
    errors: list[str] = []
    if not isinstance(data, dict):
        return ["registry root must be a JSON object"]

    for key in ("schema_version", "registry_id", "registry_status", "generated_from", "status_vocabulary", "models"):
        if key not in data:
            errors.append(f"missing required root field: {key}")

    if data.get("schema_version") != "1.0.0":
        errors.append("schema_version must be exactly '1.0.0' for this validator")
    if data.get("registry_id") != "mycelix.economy-atlas":
        errors.append("registry_id must be 'mycelix.economy-atlas'")
    vocabulary = data.get("status_vocabulary")
    if not isinstance(vocabulary, dict):
        errors.append("status_vocabulary must be a JSON object")
    elif any(status not in vocabulary for status in EPISTEMIC_STATUSES):
        errors.append("status_vocabulary must describe all allowed epistemic statuses")
    if data.get("qualified_for_execution") is not False:
        errors.append("v1 discovery registry must keep qualified_for_execution=false")

    generated = data.get("generated_from")
    if not isinstance(generated, dict):
        errors.append("generated_from must be a JSON object")
        snapshots: dict[str, object] = {}
    else:
        snapshots = generated.get("pinned_source_snapshots", {})
        if not isinstance(snapshots, dict) or not snapshots:
            errors.append("generated_from.pinned_source_snapshots must be a non-empty object")
            snapshots = {}
        else:
            for repo, sha in snapshots.items():
                if not isinstance(repo, str) or not isinstance(sha, str) or not SHA40.fullmatch(sha):
                    errors.append(f"source snapshot for {repo!r} must be a lowercase 40-hex Git SHA")

    models = data.get("models")
    if not isinstance(models, list) or not models:
        errors.append("models must be a non-empty array")
        return errors

    seen_ids: set[str] = set()
    seen_finding_ids: set[str] = set()
    present_ids: set[str] = set()
    for index, model in enumerate(models):
        prefix = f"models[{index}]"
        if not isinstance(model, dict):
            errors.append(f"{prefix} must be a JSON object")
            continue

        model_id = model.get("model_id")
        if not isinstance(model_id, str) or not model_id.strip():
            errors.append(f"{prefix}.model_id must be a non-empty string")
        elif model_id in seen_ids:
            errors.append(f"duplicate model_id: {model_id}")
        else:
            seen_ids.add(model_id)
            present_ids.add(model_id)

        for key in ("title", "execution_status"):
            if not isinstance(model.get(key), str) or not model[key].strip():
                errors.append(f"{prefix}.{key} must be a non-empty string")

        domains = model.get("domain")
        if not isinstance(domains, list) or not domains or any(not isinstance(d, str) or not d.strip() for d in domains):
            errors.append(f"{prefix}.domain must be a non-empty array of non-empty strings")

        status = model.get("epistemic_status")
        if not isinstance(status, str) or status not in EPISTEMIC_STATUSES:
            errors.append(f"{prefix}.epistemic_status has unsupported value {status!r}")

        limitations = model.get("limitations")
        if not isinstance(limitations, list) or any(not isinstance(item, str) or not item.strip() for item in limitations):
            errors.append(f"{prefix}.limitations must be an array of non-empty strings")

        sources = model.get("source_refs")
        if not isinstance(sources, list):
            errors.append(f"{prefix}.source_refs must be an array")
            continue
        if status not in {"discovery_required", "external_framework"} and not sources:
            errors.append(f"{prefix} has no sources but is not marked discovery_required/external_framework")

        # External references are a separate evidence channel: they must never be
        # misrepresented as a commit-pinned in-repository implementation source.
        research_refs = model.get("research_refs", [])
        if not isinstance(research_refs, list):
            errors.append(f"{prefix}.research_refs must be an array")
            research_refs = []
        if status == "external_framework" and not research_refs:
            errors.append(f"{prefix} external_framework requires at least one research_ref")
        seen_research_urls: set[str] = set()
        for research_index, research_ref in enumerate(research_refs):
            rp = f"{prefix}.research_refs[{research_index}]"
            if not isinstance(research_ref, dict):
                errors.append(f"{rp} must be a JSON object")
                continue
            for field in ("title", "publisher", "reference_type", "url", "retrieved_on"):
                value = research_ref.get(field)
                if not isinstance(value, str) or not value.strip():
                    errors.append(f"{rp}.{field} must be a non-empty string")
            research_url = research_ref.get("url")
            if isinstance(research_url, str):
                parsed_research_url = urlparse(research_url)
                if parsed_research_url.scheme != "https" or not parsed_research_url.netloc:
                    errors.append(f"{rp}.url must be an absolute HTTPS URL")
                if research_url in seen_research_urls:
                    errors.append(f"{rp}.url duplicates another research reference in this model")
                seen_research_urls.add(research_url)
            published_year = research_ref.get("published_year")
            if published_year is not None and (
                not isinstance(published_year, int)
                or isinstance(published_year, bool)
                or not 1800 <= published_year <= 2200
            ):
                errors.append(f"{rp}.published_year must be a plausible integer year")
            retrieved_on = research_ref.get("retrieved_on")
            if isinstance(retrieved_on, str):
                try:
                    date.fromisoformat(retrieved_on)
                except ValueError:
                    errors.append(f"{rp}.retrieved_on must be a real ISO-8601 calendar date")

        seen_source_keys: set[tuple[str, str, str]] = set()
        for source_index, source in enumerate(sources):
            sp = f"{prefix}.source_refs[{source_index}]"
            if not isinstance(source, dict):
                errors.append(f"{sp} must be a JSON object")
                continue

            repo, path, ref, url = (
                source.get("repository"),
                source.get("path"),
                source.get("ref"),
                source.get("url"),
            )
            if not isinstance(repo, str) or not re.fullmatch(r"[^/]+/[^/]+", repo):
                errors.append(f"{sp}.repository must use owner/repo format")
                continue
            if not isinstance(path, str) or not path.strip() or path.startswith("/"):
                errors.append(f"{sp}.path must be a non-empty repository-relative path")
            if not isinstance(ref, str) or not SHA40.fullmatch(ref):
                errors.append(f"{sp}.ref must be an immutable lowercase 40-hex Git commit SHA")
            if isinstance(ref, str) and snapshots.get(repo) != ref:
                errors.append(f"{sp}.ref does not match the registry's pinned snapshot for {repo}")

            if not isinstance(url, str):
                errors.append(f"{sp}.url must be a string")
            else:
                parsed = urlparse(url)
                url_parts = parsed.path.strip("/").split("/")
                if parsed.scheme != "https" or parsed.netloc != "github.com" or parsed.query or parsed.fragment:
                    errors.append(f"{sp}.url must be an HTTPS github.com source permalink")
                elif len(url_parts) < 5 or url_parts[2] != "blob":
                    errors.append(f"{sp}.url must be a GitHub /blob/<commit>/<path> permalink")
                else:
                    url_repo = "/".join(url_parts[:2])
                    url_ref = url_parts[3]
                    url_path = "/".join(url_parts[4:])
                    if url_repo != repo:
                        errors.append(f"{sp}.url repository does not match source.repository")
                    if ref != url_ref:
                        errors.append(f"{sp}.url commit does not match source.ref")
                    if path != url_path:
                        errors.append(f"{sp}.url path does not match source.path")

            if isinstance(repo, str) and isinstance(path, str) and isinstance(ref, str):
                key = (repo, path, ref)
                if key in seen_source_keys:
                    errors.append(f"{sp} duplicates the same repository/path/commit reference")
                seen_source_keys.add(key)

        findings = model.get("validation_findings", [])
        if not isinstance(findings, list):
            errors.append(f"{prefix}.validation_findings must be an array")
            findings = []
        source_triplets: set[tuple[str, str, str]] = set()
        for source in sources:
            if not isinstance(source, dict):
                continue
            repo_value = source.get("repository")
            path_value = source.get("path")
            ref_value = source.get("ref")
            if all(isinstance(value, str) for value in (repo_value, path_value, ref_value)):
                source_triplets.add((repo_value, path_value, ref_value))
        for finding_index, finding in enumerate(findings):
            fp = f"{prefix}.validation_findings[{finding_index}]"
            if not isinstance(finding, dict):
                errors.append(f"{fp} must be a JSON object")
                continue
            for field in (
                "finding_id", "severity", "source_repository", "source_file", "source_commit",
                "observation", "required_follow_up"
            ):
                value = finding.get(field)
                if not isinstance(value, str) or not value.strip():
                    errors.append(f"{fp}.{field} must be a non-empty string")
            finding_id = finding.get("finding_id")
            if isinstance(finding_id, str) and finding_id:
                if finding_id in seen_finding_ids:
                    errors.append(f"duplicate validation finding ID: {finding_id}")
                seen_finding_ids.add(finding_id)
            source_repository = finding.get("source_repository")
            source_file = finding.get("source_file")
            source_commit = finding.get("source_commit")
            if not isinstance(source_commit, str) or not SHA40.fullmatch(source_commit):
                errors.append(f"{fp}.source_commit must be a lowercase 40-hex Git commit SHA")
            if (not isinstance(source_repository, str) or not isinstance(source_file, str)
                    or not isinstance(source_commit, str)
                    or (source_repository, source_file, source_commit) not in source_triplets):
                errors.append(f"{fp} must bind to an exact source_refs repository/path/commit in the same model record")

    missing_variants = REQUIRED_VARIANTS - present_ids
    if missing_variants:
        errors.append("required semantically distinct currency variants missing: " + ", ".join(sorted(missing_variants)))

    missing_regenerative = REQUIRED_REGENERATIVE_MODELS - present_ids
    if missing_regenerative:
        errors.append("required regenerative-economics frameworks missing: " + ", ".join(sorted(missing_regenerative)))

    # This v1 registry is only a source inventory. It cannot authorize execution.
    if data.get("registry_status") != "initial_discovery_inventory":
        errors.append("v1 registry_status must remain 'initial_discovery_inventory' until a reviewed schema migration")

    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("registry", nargs="?", type=Path, default=DEFAULT_REGISTRY, help="path to model-registry.v1.json")
    args = parser.parse_args(argv)

    try:
        with args.registry.open("r", encoding="utf-8") as handle:
            registry = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"FAIL: cannot read valid registry JSON at {args.registry}: {exc}", file=sys.stderr)
        return 2

    errors = validate_registry(registry)
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        print(f"Registry rejected: {len(errors)} violation(s).", file=sys.stderr)
        return 1

    print(
        "PASS: Economy Atlas v1 registry structure and source-permalink contract "
        f"validated ({len(registry['models'])} models, "
        f"{sum(len(m['source_refs']) for m in registry['models'])} pinned source refs)."
    )
    print("NOTE: structural validation is not model execution or empirical qualification.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
