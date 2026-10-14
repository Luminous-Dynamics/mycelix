#!/usr/bin/env python3
"""Validate the Mycelix food traceability V1 schema and fail-closed invariants."""

from __future__ import annotations

import copy
import hashlib
import json
import sys
from datetime import datetime
from pathlib import Path

from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = ROOT / "docs/food/schemas/food-traceability-event.v1.schema.json"
MANIFEST_PATH = ROOT / "docs/food/food-traceability-corpus-manifest.v1.json"
EXAMPLE_PATH = ROOT / "docs/food/examples/eggs-produced.v1.json"
FORECAST_PATH = ROOT / "docs/food/examples/forecasted-harvest.v1.json"
TRANSFORMATION_PATH = ROOT / "docs/food/examples/milk-to-cheese-transformed.v1.json"
MISSING_HANDLING_PATH = ROOT / "docs/food/examples/cold-chain-missing-sample.v1.json"

# H0 fixture subset from pinned Recommendation 20. This is not a complete copy
# of the code list.
REC20_FIXTURE_CODES = {"C62", "DZN", "KGM", "LTR", "CEL"}
PRODUCT_UNIT_POLICY = {
    "urn:mycelix:product-profile:free-range-eggs-v1": {"C62", "DZN"},
    "urn:mycelix:product-profile:pasteurized-milk-v1": {"LTR"},
    "urn:mycelix:product-profile:cheese-v1": {"KGM"},
    "urn:mycelix:product-profile:strawberries-v1": {"KGM"},
}
# H0-only quantity precision policy. This constrains fixture semantics, not the
# full Rec. 20 vocabulary or every product's commercial packaging policy.
H0_MAX_FRACTIONAL_DIGITS = {"C62": 0, "DZN": 0, "KGM": 3, "LTR": 3}


def git_blob_sha1(path: Path) -> str:
    """Return Git's SHA-1 blob identifier for exact file bytes."""
    content = path.read_bytes()
    header = f"blob {len(content)}".encode("ascii") + bytes([0])
    return hashlib.sha1(header + content).hexdigest()


def main() -> int:
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    if manifest.get("profile") != "mycelix.food-traceability-corpus-manifest.v1":
        raise AssertionError("unexpected traceability corpus manifest profile")
    if manifest.get("corpusKind") != "synthetic":
        raise AssertionError("H0 fixture corpus must be explicitly synthetic")
    schema_entry = manifest.get("schema", {})
    if schema_entry.get("path") != SCHEMA_PATH.relative_to(ROOT).as_posix():
        raise AssertionError("manifest schema path mismatch")
    actual_schema_blob = git_blob_sha1(SCHEMA_PATH)
    if actual_schema_blob != schema_entry.get("gitBlobSha1"):
        raise AssertionError(
            f"schema Git blob mismatch: expected {schema_entry.get('gitBlobSha1')}, got {actual_schema_blob}"
        )
    expected_paths = {
        EXAMPLE_PATH.relative_to(ROOT).as_posix(),
        FORECAST_PATH.relative_to(ROOT).as_posix(),
        TRANSFORMATION_PATH.relative_to(ROOT).as_posix(),
        MISSING_HANDLING_PATH.relative_to(ROOT).as_posix(),
    }
    manifest_paths = {fixture.get("path") for fixture in manifest.get("fixtures", [])}
    if manifest_paths != expected_paths:
        raise AssertionError(
            f"manifest fixture set mismatch: expected {sorted(expected_paths)}, got {sorted(manifest_paths)}"
        )
    for fixture in manifest["fixtures"]:
        fixture_path = ROOT / fixture["path"]
        actual_blob = git_blob_sha1(fixture_path)
        if actual_blob != fixture.get("gitBlobSha1"):
            raise AssertionError(
                f"fixture Git blob mismatch for {fixture['path']}: expected {fixture.get('gitBlobSha1')}, got {actual_blob}"
            )
        json.loads(fixture_path.read_text(encoding="utf-8"))

    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    example = json.loads(EXAMPLE_PATH.read_text(encoding="utf-8"))
    forecast_example = json.loads(FORECAST_PATH.read_text(encoding="utf-8"))
    transformation_example = json.loads(TRANSFORMATION_PATH.read_text(encoding="utf-8"))
    missing_handling_example = json.loads(MISSING_HANDLING_PATH.read_text(encoding="utf-8"))

    Draft202012Validator.check_schema(schema)
    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    cases = 0

    def semantic_errors(instance: dict) -> list[str]:
        errors: list[str] = []
        event_type = instance.get("eventType")

        for collection_name in ("subjects", "inputs", "outputs"):
            for entry in instance.get(collection_name, []):
                quantity = entry.get("quantity")
                if quantity is None:
                    continue
                code = quantity.get("unitCode")
                profile_ref = entry.get("productProfileRef")
                quantity_value = quantity.get("value")
                max_scale = H0_MAX_FRACTIONAL_DIGITS.get(code)
                if isinstance(quantity_value, str) and "." in quantity_value and max_scale is not None:
                    scale = len(quantity_value.split(".", 1)[1])
                    if scale > max_scale:
                        errors.append(
                            f"{collection_name}: unit {code!r} permits at most "
                            f"{max_scale} fractional digits in the H0 fixture policy"
                        )
                if code not in REC20_FIXTURE_CODES:
                    errors.append(f"{collection_name}: unit code {code!r} is outside the pinned H0 fixture subset")
                allowed = PRODUCT_UNIT_POLICY.get(profile_ref)
                if allowed is None:
                    errors.append(f"{collection_name}: no pinned product-unit policy for {profile_ref!r}")
                elif code not in allowed:
                    errors.append(f"{collection_name}: unit {code!r} is not permitted for {profile_ref!r}")

        for observation in instance.get("handlingObservations", []):
            if observation.get("parameter") == "temperature" and observation.get("unitCode") != "CEL":
                errors.append("temperature observations must use CEL in this H0 profile")

        if event_type == "TRANSFORMED":
            subjects = {
                (x.get("productProfileRef"), x.get("lotRef"))
                for x in instance.get("subjects", [])
            }
            outputs = {
                (x.get("productProfileRef"), x.get("lotRef"))
                for x in instance.get("outputs", [])
            }
            if subjects != outputs:
                errors.append("transformation subjects must identify exactly the output lots")
            input_lots = {x.get("lotRef") for x in instance.get("inputs", [])}
            output_lots = {x.get("lotRef") for x in instance.get("outputs", [])}
            if input_lots & output_lots:
                errors.append("transformation must not reuse an input lot identity as an output lot")

        forecast = instance.get("forecastMetadata")
        if forecast:
            try:
                start = datetime.fromisoformat(forecast["horizonStart"].replace("Z", "+00:00"))
                end = datetime.fromisoformat(forecast["horizonEnd"].replace("Z", "+00:00"))
                issued = datetime.fromisoformat(forecast["issuedAt"].replace("Z", "+00:00"))
                if end <= start:
                    errors.append("forecast horizonEnd must be later than horizonStart")
                if start < issued:
                    errors.append("forecast horizonStart must not precede issuedAt")
            except (KeyError, ValueError, TypeError) as exc:
                errors.append(f"forecast timestamps are not comparable ISO datetimes: {exc}")

        if event_type == "DISPATCHED" and instance.get("siteRef") != instance.get("originSiteRef"):
            errors.append("DISPATCHED siteRef must match originSiteRef")
        if event_type == "RECEIVED" and instance.get("siteRef") != instance.get("destinationSiteRef"):
            errors.append("RECEIVED siteRef must match destinationSiteRef")
        if instance.get("supersedesEventId") == instance.get("eventId"):
            errors.append("an event must not supersede itself")
        return errors

    def event_batch_errors(events: list[dict]) -> list[str]:
        """Check event-ID equivocation and cycles in the supersession subgraph."""
        errors: list[str] = []
        canonical_content_by_id: dict[str, str] = {}
        supersedes_by_id: dict[str, str] = {}

        for event in events:
            event_id = event.get("eventId")
            if not isinstance(event_id, str):
                continue
            canonical = json.dumps(event, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
            prior = canonical_content_by_id.get(event_id)
            if prior is not None and prior != canonical:
                errors.append(f"eventId {event_id!r} was reused with different canonical content")
            else:
                canonical_content_by_id[event_id] = canonical

            target = event.get("supersedesEventId")
            if isinstance(target, str) and target:
                supersedes_by_id[event_id] = target

        # In a simple one-leg trace, a matching dispatch/receipt pair must preserve
        # the same shipment route. Multi-leg semantics need an explicit leg identity
        # before stronger assumptions can safely be made.
        shipments: dict[str, list[dict]] = {}
        seen_shipment_event_ids: set[str] = set()
        for event in events:
            event_id = event.get("eventId")
            if isinstance(event_id, str) and event_id in seen_shipment_event_ids:
                # An identical replay is one logical event for route checks. A
                # conflicting reuse is already reported above and still fails closed.
                continue
            if isinstance(event_id, str):
                seen_shipment_event_ids.add(event_id)
            shipment_ref = event.get("shipmentRef")
            if event.get("eventType") in {"DISPATCHED", "RECEIVED"} and isinstance(shipment_ref, str):
                shipments.setdefault(shipment_ref, []).append(event)
        for shipment_ref, shipment_events in shipments.items():
            dispatches = [event for event in shipment_events if event.get("eventType") == "DISPATCHED"]
            receipts = [event for event in shipment_events if event.get("eventType") == "RECEIVED"]
            if len(dispatches) == 1 and len(receipts) == 1:
                dispatched = dispatches[0]
                received = receipts[0]
                dispatch_route = (dispatched.get("originSiteRef"), dispatched.get("destinationSiteRef"))
                receipt_route = (received.get("originSiteRef"), received.get("destinationSiteRef"))
                if dispatch_route != receipt_route:
                    errors.append(
                        f"shipmentRef {shipment_ref!r} has mismatched dispatch/receipt routes"
                    )

        state: dict[str, int] = {}

        def visit(event_id: str) -> None:
            if state.get(event_id) == 1:
                errors.append(f"supersession cycle includes eventId {event_id!r}")
                return
            if state.get(event_id) == 2:
                return
            state[event_id] = 1
            target = supersedes_by_id.get(event_id)
            if target in supersedes_by_id:
                visit(target)
            state[event_id] = 2

        for event_id in supersedes_by_id:
            visit(event_id)
        return sorted(set(errors))

    def downstream_lot_closure(events: list[dict], roots: set[str]) -> set[str]:
        """Return recall candidates reachable only downstream through transformations."""
        outputs_by_input: dict[str, set[str]] = {}
        for event in events:
            if event.get("eventType") != "TRANSFORMED":
                continue
            input_lots = {
                entry.get("lotRef")
                for entry in event.get("inputs", [])
                if isinstance(entry.get("lotRef"), str)
            }
            output_lots = {
                entry.get("lotRef")
                for entry in event.get("outputs", [])
                if isinstance(entry.get("lotRef"), str)
            }
            for input_lot in input_lots:
                outputs_by_input.setdefault(input_lot, set()).update(output_lots)

        affected = set(roots)
        pending = list(roots)
        while pending:
            current = pending.pop()
            for downstream in outputs_by_input.get(current, set()):
                if downstream not in affected:
                    affected.add(downstream)
                    pending.append(downstream)
        return affected

    def valid_batch(name: str, events: list[dict]) -> None:
        nonlocal cases
        cases += 1
        failures: list[str] = []
        for index, instance in enumerate(events):
            schema_errors = list(validator.iter_errors(instance))
            semantic = semantic_errors(instance)
            if schema_errors or semantic:
                failures.append(f"event {index}: schema_errors={len(schema_errors)} semantic={semantic}")
        batch_errors = event_batch_errors(events)
        failures.extend(batch_errors)
        if failures:
            raise AssertionError(f"{name} should be valid:\n  " + "\n  ".join(failures))
        print(f"PASS valid batch: {name}")

    def invalid_batch(name: str, events: list[dict]) -> None:
        nonlocal cases
        cases += 1
        batch_errors = event_batch_errors(events)
        if not batch_errors:
            # A batch-level negative case must exercise the batch invariants, not
            # merely fail an unrelated single-event schema check.
            raise AssertionError(f"{name} should fail a batch invariant but did not")
        print(f"PASS rejects batch: {name}")

    def valid(name: str, instance: dict) -> None:
        nonlocal cases
        cases += 1
        errors = sorted(validator.iter_errors(instance), key=lambda err: list(map(str, err.path)))
        semantic = semantic_errors(instance)
        if errors or semantic:
            formatted = "\n".join(f"  {list(err.path)}: {err.message}" for err in errors)
            formatted += ("\n" if formatted and semantic else "") + "\n".join(f"  semantic: {message}" for message in semantic)
            raise AssertionError(f"{name} should be valid:\n{formatted}")
        print(f"PASS valid: {name}")

    def invalid(name: str, instance: dict) -> None:
        nonlocal cases
        cases += 1
        errors = list(validator.iter_errors(instance))
        semantic = semantic_errors(instance)
        if not errors and not semantic:
            raise AssertionError(f"{name} should be invalid but validated successfully")
        print(f"PASS rejects: {name}")

    valid("canonical synthetic egg lot by dozen", example)

    eggs_by_count = copy.deepcopy(example)
    eggs_by_count["subjects"][0]["quantity"]["value"] = "96"
    eggs_by_count["subjects"][0]["quantity"]["unitCode"] = "C62"
    valid("eggs may also be recorded by individual count", eggs_by_count)
    valid("forecast binds model identity, version, and horizon", forecast_example)
    valid("dairy transformation binds observed input and output lots", transformation_example)
    valid("missing cold-chain sample remains explicitly unknown", missing_handling_example)

    transformation_second = copy.deepcopy(transformation_example)
    transformation_second["eventId"] = "urn:mycelix:food-event:processor-01:cheese-to-packed-cheese-20261012-002"
    transformation_second["eventTime"] = "2026-10-12T09:00:00+02:00"
    transformation_second["recordedAt"] = "2026-10-12T09:02:00+02:00"
    transformation_second["inputs"] = copy.deepcopy(transformation_example["outputs"])
    transformation_second["outputs"] = [{
        "productProfileRef": "urn:mycelix:product-profile:cheese-v1",
        "lotRef": "urn:mycelix:food-lot:cheese-packed-20261012-a",
        "quantity": {
            "status": "observed",
            "value": "3",
            "unitCode": "KGM",
            "unitCodeSystem": "UNECE_REC20",
            "unitCodeListVersion": "REC20-REV17-2021",
        },
    }]
    transformation_second["subjects"] = [{
        "productProfileRef": "urn:mycelix:product-profile:cheese-v1",
        "lotRef": "urn:mycelix:food-lot:cheese-packed-20261012-a",
    }]
    valid("second transformation creates a distinct downstream lot", transformation_second)

    milk_lot = "urn:mycelix:food-lot:milk-20261012-a"
    cheese_lot = "urn:mycelix:food-lot:cheese-20261012-a"
    packed_lot = "urn:mycelix:food-lot:cheese-packed-20261012-a"
    recall = copy.deepcopy(example)
    recall["eventId"] = "urn:mycelix:food-event:operator-01:recall-milk-20261012-001"
    recall["eventType"] = "RECALLED"
    recall["subjects"] = [{
        "productProfileRef": "urn:mycelix:product-profile:pasteurized-milk-v1",
        "lotRef": milk_lot,
    }]
    recall["eventTime"] = "2026-10-12T10:00:00+02:00"
    recall["recordedAt"] = "2026-10-12T10:02:00+02:00"
    valid("recall identifies input lot without inventing a quantity", recall)
    valid_batch(
        "recall source, intermediate cheese and packed descendants form valid event batch",
        [transformation_example, transformation_second, recall],
    )
    affected_lots = downstream_lot_closure(
        [transformation_example, transformation_second],
        {milk_lot},
    )
    if affected_lots != {milk_lot, cheese_lot, packed_lot}:
        raise AssertionError(
            f"recall must reach every downstream lot; expected "
            f"{sorted({milk_lot, cheese_lot, packed_lot})}, got {sorted(affected_lots)}"
        )
    cases += 1
    print("PASS recall closure: input recall reaches intermediate and packed descendants")

    downstream_from_cheese = downstream_lot_closure(
        [transformation_example, transformation_second],
        {cheese_lot},
    )
    if downstream_from_cheese != {cheese_lot, packed_lot}:
        raise AssertionError(
            "recall traversal must be directed: recalling an output must not propagate backward to its inputs"
        )
    cases += 1
    print("PASS recall negative control: downstream-only traversal does not mark upstream milk")

    unknown = copy.deepcopy(example)
    unknown["subjects"][0]["quantity"]["status"] = "unknown"
    unknown["subjects"][0]["quantity"].pop("value")
    valid("unknown amount carries no numeric value", unknown)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["status"] = "unknown"
    invalid("unknown amount cannot smuggle a numeric value", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["value"] = "-1"
    invalid("negative amount", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["value"] = "1e3"
    invalid("exponent notation is not canonical decimal", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["value"] = "8.0"
    invalid("canonical decimal must not contain trailing fractional zeroes", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["unitCode"] = "C62"
    bad["subjects"][0]["quantity"]["value"] = "96.5"
    invalid("individual egg count must be a whole item in the H0 policy", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["unitCode"] = "KGM"
    bad["subjects"][0]["quantity"]["value"] = "1.2345"
    invalid("mass quantity exceeds the H0 three-decimal precision ceiling", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["value"] = "NaN"
    invalid("NaN is not a quantity", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["unitCode"] = "dzn"
    invalid("unit codes use canonical uppercase syntax", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["unitCode"] = "XYZ"
    invalid("unit code must be in the pinned H0 fixture subset", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["unitCode"] = "KGM"
    invalid("product profile cannot use an incompatible quantity unit", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["unitCodeListVersion"] = "latest"
    invalid("unit-code vocabulary release must be pinned", bad)

    bad = copy.deepcopy(example)
    bad["unexpected"] = "must not be silently accepted"
    invalid("unknown top-level property", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "PLANNED"
    invalid("planned event cannot use observed quantity state", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "FORECASTED"
    invalid("forecast event cannot use observed quantity state", bad)

    bad = copy.deepcopy(example)
    bad["subjects"][0]["quantity"]["status"] = "forecast"
    invalid("observed production event cannot carry forecast quantity state", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "TRANSFORMED"
    invalid("transformation requires explicit input and output lots", bad)

    bad = copy.deepcopy(transformation_example)
    bad["inputs"][0]["quantity"]["status"] = "forecast"
    invalid("transformation input cannot be forecast rather than observed/unknown", bad)

    bad = copy.deepcopy(transformation_example)
    bad["eventType"] = "PRODUCED"
    invalid("non-transformation event cannot carry transformation inputs/outputs", bad)

    bad = copy.deepcopy(transformation_example)
    bad["subjects"][0]["lotRef"] = "urn:mycelix:food-lot:other-cheese"
    invalid("transformation subjects must match output lots", bad)

    bad = copy.deepcopy(transformation_example)
    bad["outputs"][0]["lotRef"] = bad["inputs"][0]["lotRef"]
    bad["subjects"][0]["lotRef"] = bad["outputs"][0]["lotRef"]
    bad["subjects"][0]["productProfileRef"] = bad["inputs"][0]["productProfileRef"]
    invalid("transformation cannot reuse an input lot identity as output", bad)

    bad = copy.deepcopy(forecast_example)
    bad["forecastMetadata"].pop("modelVersion")
    invalid("forecast requires a model version", bad)

    bad = copy.deepcopy(forecast_example)
    bad["eventType"] = "PRODUCED"
    invalid("non-forecast event cannot carry forecast metadata", bad)

    bad = copy.deepcopy(forecast_example)
    bad["forecastMetadata"]["horizonEnd"] = "2026-10-12T23:59:59+02:00"
    invalid("forecast horizon must be ordered and not predate issuance", bad)

    dispatch = copy.deepcopy(example)
    dispatch["eventId"] = "urn:mycelix:food-event:producer-01:eggs-lot-20261012:dispatched-001"
    dispatch["eventType"] = "DISPATCHED"
    dispatch["shipmentRef"] = "urn:mycelix:shipment:run-20261012-01"
    dispatch["originSiteRef"] = dispatch["siteRef"]
    dispatch["destinationSiteRef"] = "urn:mycelix:site:cooperative-hub-01"
    valid("dispatch binds shipment and both endpoints", dispatch)

    bad = copy.deepcopy(dispatch)
    bad.pop("shipmentRef")
    invalid("dispatch cannot omit shipment identity", bad)

    bad = copy.deepcopy(dispatch)
    bad["originSiteRef"] = "urn:mycelix:site:wrong-origin"
    invalid("dispatch site must agree with shipment origin", bad)

    received = copy.deepcopy(example)
    received["eventId"] = "urn:mycelix:food-event:cooperative-hub-01:eggs-lot-20261012:received-001"
    received["eventType"] = "RECEIVED"
    received["shipmentRef"] = dispatch["shipmentRef"]
    received["originSiteRef"] = dispatch["originSiteRef"]
    received["destinationSiteRef"] = "urn:mycelix:site:cooperative-hub-01"
    received["siteRef"] = received["destinationSiteRef"]
    valid("receipt binds shipment and agrees with destination site", received)
    valid_batch("matching dispatch and receipt preserve one shipment route", [dispatch, received])

    bad_receipt_route = copy.deepcopy(received)
    bad_receipt_route["originSiteRef"] = "urn:mycelix:site:unexpected-origin"
    invalid_batch("dispatch and receipt for one simple shipment must agree on route",
                  [dispatch, bad_receipt_route])

    bad = copy.deepcopy(received)
    bad["siteRef"] = "urn:mycelix:site:unrelated-location"
    invalid("receipt site must agree with shipment destination", bad)

    bad = copy.deepcopy(received)
    bad["destinationSiteRef"] = "urn:mycelix:site:wrong-destination"
    invalid("receipt cannot claim a conflicting destination", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "HANDLING_OBSERVED"
    bad["subjects"][0].pop("quantity")
    invalid("handling event requires handling observations", bad)

    handling_identity_only = copy.deepcopy(example)
    handling_identity_only["eventType"] = "HANDLING_OBSERVED"
    handling_identity_only["subjects"][0].pop("quantity")
    handling_identity_only["handlingObservations"] = [{
        "parameter": "temperature",
        "status": "missing",
        "observedAt": "2026-10-12T06:01:00+02:00",
        "receivedAt": "2026-10-12T06:01:05+02:00",
        "sensorRef": "urn:mycelix:sensor:cold-room-01",
        "calibrationStatus": "unknown",
        "unitCode": "CEL",
        "unitCodeSystem": "UNECE_REC20",
        "unitCodeListVersion": "REC20-REV17-2021"
    }]
    valid("handling observation identifies lot without inventing a quantity", handling_identity_only)

    handling_base = copy.deepcopy(example)
    handling_base["eventType"] = "HANDLING_OBSERVED"
    handling_base["subjects"][0].pop("quantity")
    handling = {
        "parameter": "temperature",
        "status": "observed",
        "value": "4.2",
        "observedAt": "2026-10-12T06:01:00+02:00",
        "receivedAt": "2026-10-12T06:01:05+02:00",
        "sensorRef": "urn:mycelix:sensor:cold-room-01",
        "calibrationStatus": "current",
        "calibrationRef": "urn:mycelix:calibration:cold-room-01:2026-09",
        "unitCode": "CEL",
        "unitCodeSystem": "UNECE_REC20",
        "unitCodeListVersion": "REC20-REV17-2021",
    }
    handling_base["handlingObservations"] = [handling]
    valid("well-formed observed handling measurement", handling_base)

    bad = copy.deepcopy(handling_base)
    bad["handlingObservations"][0].pop("value")
    invalid("observed handling measurement requires a value", bad)

    bad = copy.deepcopy(handling_base)
    bad["handlingObservations"][0].pop("calibrationRef")
    invalid("current calibration status requires calibration evidence reference", bad)

    bad = copy.deepcopy(handling_base)
    bad["handlingObservations"][0]["status"] = "missing"
    invalid("missing handling sample cannot claim a value", bad)

    missing_with_unknown_value = copy.deepcopy(missing_handling_example)
    missing_with_unknown_value["handlingObservations"][0]["value"] = "4.0"
    invalid("missing cold-chain sample cannot smuggle a numeric reading", missing_with_unknown_value)

    bad = copy.deepcopy(example)
    bad["evidenceRefs"][0]["status"] = "verified"
    invalid("verified evidence requires verifier and procedure references", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "CORRECTION"
    invalid("correction requires supersession link and reason", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "RETRACTION"
    bad["supersedesEventId"] = example["eventId"]
    bad["correctionReason"] = "Test self-reference rejection."
    invalid("retraction cannot supersede itself even when its reason is present", bad)

    bad = copy.deepcopy(example)
    bad["eventType"] = "RETRACTION"
    bad["supersedesEventId"] = "urn:mycelix:food-event:producer-01:prior-event"
    invalid("retraction requires a non-empty reason", bad)

    correction = copy.deepcopy(example)
    correction["eventId"] = "urn:mycelix:food-event:producer-01:eggs-lot-20261012:correction-001"
    correction["eventType"] = "CORRECTION"
    correction["eventTime"] = "2026-10-13T11:00:00+02:00"
    correction["recordedAt"] = "2026-10-13T11:00:05+02:00"
    correction["subjects"][0]["quantity"]["value"] = "7"
    correction["supersedesEventId"] = example["eventId"]
    correction["correctionReason"] = "Corrected the reported eight dozen to seven dozen against the producer's source ledger."
    valid("correction appends a reasoned reference to the prior event", correction)
    valid_batch("original event followed by a valid correction", [example, correction])

    retraction = copy.deepcopy(example)
    retraction["eventId"] = "urn:mycelix:food-event:producer-01:eggs-lot-20261012:retraction-001"
    retraction["eventType"] = "RETRACTION"
    retraction["eventTime"] = "2026-10-13T11:30:00+02:00"
    retraction["recordedAt"] = "2026-10-13T11:30:05+02:00"
    retraction["supersedesEventId"] = example["eventId"]
    retraction["correctionReason"] = "Source record withdrawn by its issuer."
    valid("retraction carries a supersession link and reason", retraction)

    bad = copy.deepcopy(example)
    bad["supersedesEventId"] = "urn:mycelix:food-event:producer-01:prior-event"
    invalid("ordinary production event cannot carry supersession metadata", bad)

    valid_batch("same event ID and canonical content is an idempotent replay",
                [example, copy.deepcopy(example)])

    replay_conflict = copy.deepcopy(example)
    replay_conflict["notes"] = "Different claim under a reused event identity."
    invalid_batch("same event ID with different content is a conflict",
                  [example, replay_conflict])

    correction_a = copy.deepcopy(correction)
    correction_a["eventId"] = "urn:mycelix:food-event:producer-01:correction-cycle-a"
    correction_b = copy.deepcopy(correction)
    correction_b["eventId"] = "urn:mycelix:food-event:producer-01:correction-cycle-b"
    correction_a["supersedesEventId"] = correction_b["eventId"]
    correction_b["supersedesEventId"] = correction_a["eventId"]
    invalid_batch("mutually superseding corrections form a rejected cycle",
                  [correction_a, correction_b])

    print(f"PASS: JSON Schema draft 2020-12 meta-validation and {cases} contract cases")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, json.JSONDecodeError, OSError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
