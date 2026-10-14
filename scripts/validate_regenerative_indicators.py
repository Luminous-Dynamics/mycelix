#!/usr/bin/env python3
"""Fail-closed structural and provenance checks for the regenerative indicator catalog.

This script validates definitions and metadata only. It does not fetch measurements,
calculate indicators, establish causality, or certify regenerative outcomes.
"""
from __future__ import annotations

import json
import sys
from datetime import date
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]
CATALOG_PATH = ROOT / "docs" / "economy-atlas" / "regenerative-indicators.v1.json"
REGISTRY_PATH = ROOT / "docs" / "economy-atlas" / "model-registry.v1.json"
DIRECTIONS = {
    "higher_is_better",
    "lower_is_better",
    "context_dependent",
    "target_band",
    "boundary_ratio",
    "descriptive_only",
}
INDICATOR_TYPES = {
    "observed",
    "derived",
    "observed_or_derived",
    "observed_or_accounted",
    "accounting_series",
    "derived_accounting_series",
    "network_metric",
}
REQUIRED_INDICATORS = {
    "social.basic-needs-deprivation-share.v1",
    "ecology.ecosystem-extent-by-type.v1",
    "ecology.ecosystem-condition-profile.v1",
    "ecology.physical-ecosystem-service-flow.v1",
    "climate.absolute-territorial-ghg.v1",
    "climate.territorial-ghg-per-capita.v1",
    "water.pollutant-concentration.v1",
    "water.physicochemical-profile.v1",
    "water.microbial-contamination.v1",
    "water.toxic-elements-concentration.v1",
    "soil.organic-carbon-stock.v1",
    "soil.water-infiltration-rate.v1",
    "biodiversity.native-taxon-occupancy.v1",
    "materials.material-footprint-per-capita.v1",
    "materials.virgin-material-input.v1",
    "circularity.product-material-circularity.v1",
    "circularity.material-repair-reuse-share.v1",
    "waste.waste-generation-per-capita.v1",
    "agroecology.yield-stability.v1",
    "network.finn-cycling-index.v1",
    "network.energy-network-structure.v1",
}
REQUIRED_INDICATOR_FIELDS = (
    "indicator_id", "title", "framework_ids", "domain", "indicator_type",
    "definition", "native_unit", "denominator", "desired_direction",
    "spatial_scope_required", "temporal_resolution_required",
    "baseline_and_target", "uncertainty_required", "input_requirements",
    "comparability_key", "missing_data_rule", "caveats", "research_ref_ids",
    "status", "observations_loaded", "computed",
)


def validate_catalog(catalog: dict, model_registry: dict) -> list[str]:
    if not isinstance(catalog, dict):
        return ["catalog root must be an object"]
    errors: list[str] = []
    if not isinstance(model_registry, dict):
        errors.append("model registry root must be an object")
        model_registry = {}
    if catalog.get("schema_version") != "1.0.0":
        errors.append("schema_version must be 1.0.0")
    if catalog.get("catalog_id") != "mycelix.regenerative-indicators":
        errors.append("catalog_id must be mycelix.regenerative-indicators")
    if catalog.get("catalog_status") != "candidate_indicator_definitions":
        errors.append("catalog_status must remain candidate_indicator_definitions")

    qualification = catalog.get("qualification")
    expected_qualification = {
        "empirical_observations_loaded": False,
        "indicators_computed": False,
        "independently_validated": False,
        "composite_score_enabled": False,
    }
    if qualification != expected_qualification:
        errors.append("qualification must keep all evidence/execution flags false")

    comparisons = catalog.get("global_comparison_contract")
    if not isinstance(comparisons, list) or len(comparisons) < 5 or any(
        not isinstance(x, str) or len(x.strip()) < 10 for x in comparisons
    ):
        errors.append("global_comparison_contract must contain substantive rules")

    missingness = catalog.get("missingness_vocabulary")
    if not isinstance(missingness, list) or not missingness or any(
        not isinstance(value, str) or not value.strip() for value in missingness
    ):
        errors.append("missingness_vocabulary must be a non-empty string list")
    elif len(missingness) != len(set(missingness)):
        errors.append("missingness_vocabulary must not contain duplicates")
    elif not {"unknown", "not_collected", "not_computable"}.issubset(set(missingness)):
        errors.append("missingness_vocabulary must distinguish unknown, not_collected, and not_computable")

    references = catalog.get("research_refs")
    if not isinstance(references, list) or not references:
        errors.append("research_refs must be a non-empty list")
        references = []
    ref_ids: set[str] = set()
    for index, ref in enumerate(references):
        if not isinstance(ref, dict):
            errors.append(f"research_refs[{index}] must be an object")
            continue
        ref_id = ref.get("ref_id")
        if not isinstance(ref_id, str) or not ref_id:
            errors.append(f"research_refs[{index}] has no ref_id")
        elif ref_id in ref_ids:
            errors.append(f"duplicate research ref_id: {ref_id}")
        else:
            ref_ids.add(ref_id)
        for field in ("title", "publisher", "reference_type"):
            if not isinstance(ref.get(field), str) or len(ref[field].strip()) < 2:
                errors.append(f"research_refs[{index}].{field} must be a non-empty string")
        url = ref.get("url")
        parsed = urlparse(url) if isinstance(url, str) else None
        if parsed is None or parsed.scheme != "https" or not parsed.hostname:
            errors.append(f"research_refs[{index}].url must be HTTPS with a hostname")
        retrieved_on = ref.get("retrieved_on")
        try:
            if not isinstance(retrieved_on, str) or date.fromisoformat(retrieved_on).isoformat() != retrieved_on:
                raise ValueError("not canonical ISO date")
        except ValueError:
            errors.append(f"research_refs[{index}].retrieved_on must be a real YYYY-MM-DD date")
        year = ref.get("published_year")
        if year is not None and (not isinstance(year, int) or year < 1900 or year > 2100):
            errors.append(f"research_refs[{index}].published_year must be a plausible year")

    model_records = model_registry.get("models", [])
    if not isinstance(model_records, list):
        errors.append("model registry models must be a list")
        model_records = []
    known_models = {
        model.get("model_id")
        for model in model_records
        if isinstance(model, dict) and isinstance(model.get("model_id"), str)
    }
    indicators = catalog.get("indicators")
    if not isinstance(indicators, list) or not indicators:
        errors.append("indicators must be a non-empty list")
        indicators = []
    ids: set[str] = set()
    found_ids: set[str] = set()
    for index, indicator in enumerate(indicators):
        where = f"indicators[{index}]"
        if not isinstance(indicator, dict):
            errors.append(f"{where} must be an object")
            continue
        indicator_id = indicator.get("indicator_id")
        if not isinstance(indicator_id, str) or not indicator_id:
            errors.append(f"{where}.indicator_id must be a non-empty string")
        elif indicator_id in ids:
            errors.append(f"duplicate indicator_id: {indicator_id}")
        else:
            ids.add(indicator_id)
            found_ids.add(indicator_id)
        for field in REQUIRED_INDICATOR_FIELDS:
            if field not in indicator:
                errors.append(f"{where} is missing required field {field}")
        for field in (
            "title", "domain", "definition", "native_unit", "denominator",
            "spatial_scope_required", "temporal_resolution_required",
            "baseline_and_target", "uncertainty_required", "missing_data_rule",
        ):
            if field in indicator and (
                not isinstance(indicator[field], str) or not indicator[field].strip()
            ):
                errors.append(f"{where}.{field} must be a non-empty string")
        if indicator.get("status") != "candidate_definition":
            errors.append(f"{where}.status must remain candidate_definition")
        if indicator.get("observations_loaded") is not False:
            errors.append(f"{where}.observations_loaded must be false until evidence is attached")
        if indicator.get("computed") is not False:
            errors.append(f"{where}.computed must be false until a qualified runner exists")
        direction = indicator.get("desired_direction")
        if not isinstance(direction, str) or direction not in DIRECTIONS:
            errors.append(f"{where}.desired_direction is not an allowed value")
        indicator_type = indicator.get("indicator_type")
        if not isinstance(indicator_type, str) or indicator_type not in INDICATOR_TYPES:
            errors.append(f"{where}.indicator_type is not an allowed value")

        for field in ("framework_ids", "input_requirements", "comparability_key", "caveats", "research_ref_ids"):
            values = indicator.get(field)
            if not isinstance(values, list) or not values or any(not isinstance(v, str) or not v.strip() for v in values):
                errors.append(f"{where}.{field} must be a non-empty string list")
                continue
            if field in {"framework_ids", "comparability_key", "research_ref_ids"} and len(values) != len(set(values)):
                errors.append(f"{where}.{field} must not contain duplicates")
            if field == "framework_ids":
                unknown = sorted(set(values) - known_models)
                if unknown:
                    errors.append(f"{where}.framework_ids reference unknown model IDs: {unknown}")
            if field == "research_ref_ids":
                unknown = sorted(set(values) - ref_ids)
                if unknown:
                    errors.append(f"{where}.research_ref_ids reference unknown research refs: {unknown}")

    missing = sorted(REQUIRED_INDICATORS - found_ids)
    if missing:
        errors.append(f"required indicator definitions missing: {missing}")
    if len(ids) != len(indicators):
        errors.append("indicator IDs must be unique and all entries must be objects")
    return errors


def main() -> int:
    try:
        catalog = json.loads(CATALOG_PATH.read_text(encoding="utf-8"))
        model_registry = json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"FAIL: cannot read indicator catalog or model registry: {exc}", file=sys.stderr)
        return 1
    errors = validate_catalog(catalog, model_registry)
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        print(f"Catalog rejected: {len(errors)} violation(s).", file=sys.stderr)
        return 1
    print(
        "PASS: regenerative indicator catalog structure and provenance validated "
        f"({len(catalog['indicators'])} candidate indicators, "
        f"{len(catalog['research_refs'])} references)."
    )
    print("NOTE: no observations were loaded, no indicators computed, and no outcome was validated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
