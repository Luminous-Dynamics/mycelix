"""Fail-closed tests for the test-only OCEL 2.0 normalizer."""

from __future__ import annotations

import json
import pathlib
import tempfile
import unittest

from pm4py_ocel2_roundtrip import (
    declared_types,
    normalize_inventory,
    normalize_time,
    typed_value,
)


def minimal_ocel() -> dict[str, object]:
    return {
        "objectTypes": [
            {"name": "Asset", "attributes": [{"name": "status", "type": "string"}]}
        ],
        "eventTypes": [
            {"name": "inspection", "attributes": [{"name": "passed", "type": "boolean"}]}
        ],
        "objects": [
            {
                "id": "asset-1",
                "type": "Asset",
                "attributes": [
                    {"name": "status", "time": "2026-11-27T08:00:00Z", "value": "open"}
                ],
                "relationships": [],
            }
        ],
        "events": [
            {
                "id": "event-1",
                "type": "inspection",
                "time": "2026-11-27T08:01:00Z",
                "attributes": [{"name": "passed", "value": False}],
                "relationships": [{"objectId": "asset-1", "qualifier": "inspects"}],
            }
        ],
    }


class StrictValueTests(unittest.TestCase):
    def test_integer_does_not_coerce_or_truncate(self) -> None:
        with self.assertRaisesRegex(ValueError, "expected integer"):
            typed_value(1.9, "integer")
        with self.assertRaisesRegex(ValueError, "expected integer"):
            typed_value("12", "integer")
        self.assertEqual(typed_value(12, "integer"), 12)

    def test_boolean_does_not_coerce_truthy_strings(self) -> None:
        with self.assertRaisesRegex(ValueError, "expected boolean"):
            typed_value("false", "boolean")
        self.assertIs(typed_value(False, "boolean"), False)
        self.assertIs(typed_value(True, "boolean"), True)

    def test_string_and_float_require_json_primitive_types(self) -> None:
        with self.assertRaisesRegex(ValueError, "expected string"):
            typed_value(123, "string")
        with self.assertRaisesRegex(ValueError, "expected numeric"):
            typed_value("1.25", "float")
        self.assertEqual(typed_value(1.25, "float"), 1.25)

    def test_timestamp_requires_offset_and_valid_rfc3339_shape(self) -> None:
        self.assertEqual(
            normalize_time("2026-11-27T10:00:00+02:00"),
            "2026-11-27T08:00:00Z",
        )
        for value in ("not-a-time", "2026-11-27T08:00:00", "2026-11-27 08:00:00Z"):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "timestamp"):
                normalize_time(value)

    def test_timestamp_rejects_precision_python_cannot_preserve(self) -> None:
        with self.assertRaisesRegex(ValueError, "timestamp"):
            normalize_time("2026-11-27T08:00:00.123456789Z")

    def test_float_rejects_non_finite_values(self) -> None:
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "finite"):
                typed_value(value, "float")


class SchemaAndInventoryTests(unittest.TestCase):
    def test_decoder_rejects_duplicate_json_members(self) -> None:
        serialized = json.dumps(minimal_ocel())
        serialized = serialized[:-1] + ',"events":[]}'
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "fixture.json"
            path.write_text(serialized, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate JSON object member: events"):
                normalize_inventory(path)

    def test_decoder_rejects_nonstandard_nan_token(self) -> None:
        serialized = json.dumps(minimal_ocel()).replace(
            '"time": "2026-11-27T08:01:00Z"', '"time": NaN', 1
        )
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "fixture.json"
            path.write_text(serialized, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-standard JSON numeric token"):
                normalize_inventory(path)

    def test_duplicate_event_relationship_is_rejected(self) -> None:
        data = minimal_ocel()
        relationship = data["events"][0]["relationships"][0]
        data["events"][0]["relationships"].append(dict(relationship))
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "fixture.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate event relationship"):
                normalize_inventory(path)

    def test_duplicate_object_relationship_is_rejected(self) -> None:
        data = minimal_ocel()
        data["objects"].append(
            {"id": "asset-2", "type": "Asset", "attributes": [], "relationships": []}
        )
        relationship = {"objectId": "asset-2", "qualifier": "associated"}
        data["objects"][0]["relationships"] = [relationship, dict(relationship)]
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "fixture.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate object relationship"):
                normalize_inventory(path)

    def test_duplicate_type_declaration_is_rejected(self) -> None:
        data = {
            "objectTypes": [
                {"name": "Asset", "attributes": []},
                {"name": "Asset", "attributes": []},
            ]
        }
        with self.assertRaisesRegex(ValueError, "duplicate objectTypes"):
            declared_types(data, "objectTypes")

    def test_duplicate_attribute_declaration_is_rejected(self) -> None:
        data = {
            "eventTypes": [
                {
                    "name": "inspection",
                    "attributes": [
                        {"name": "passed", "type": "boolean"},
                        {"name": "passed", "type": "string"},
                    ],
                }
            ]
        }
        with self.assertRaisesRegex(ValueError, "duplicate attribute declaration"):
            declared_types(data, "eventTypes")

    def test_undeclared_attribute_is_rejected(self) -> None:
        data = minimal_ocel()
        data["events"][0]["attributes"].append({"name": "unexpected", "value": "x"})
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "fixture.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "undeclared event attribute"):
                normalize_inventory(path)

    def test_unsupported_extension_field_is_not_silently_dropped(self) -> None:
        data = minimal_ocel()
        data["vendor_extension"] = {"opaque": "value"}
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "fixture.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unsupported extension field"):
                normalize_inventory(path)

    def test_repeated_object_attribute_names_preserve_temporal_history(self) -> None:
        data = minimal_ocel()
        data["objects"][0]["attributes"].append(
            {"name": "status", "time": "2026-11-27T09:00:00Z", "value": "closed"}
        )
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "fixture.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            normalized = normalize_inventory(path)
        history = normalized["objects"][0][2]
        self.assertEqual(len(history), 2)
        self.assertEqual([entry[0] for entry in history], ["status", "status"])
        self.assertNotEqual(history[0][1], history[1][1])

    def test_duplicate_object_identity_and_dangling_relationship_fail(self) -> None:
        for mutation, expected in (
            (
                lambda data: data["objects"].append(dict(data["objects"][0])),
                "duplicate object ID",
            ),
            (
                lambda data: data["events"][0]["relationships"][0].update(
                    {"objectId": "missing-object"}
                ),
                "unknown event relationship target",
            ),
        ):
            data = minimal_ocel()
            mutation(data)
            with tempfile.TemporaryDirectory() as directory:
                path = pathlib.Path(directory) / "fixture.json"
                path.write_text(json.dumps(data), encoding="utf-8")
                with self.subTest(expected=expected), self.assertRaisesRegex(ValueError, expected):
                    normalize_inventory(path)


if __name__ == "__main__":
    unittest.main()
