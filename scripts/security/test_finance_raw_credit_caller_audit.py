#!/usr/bin/env python3
# Copyright (C) 2024-2026 Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Regression tests for the fail-closed Finance raw-credit caller audit."""

from __future__ import annotations

import io
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from finance_raw_credit_caller_audit import audit, has_public_raw_credit_abi, scan


def seed_project(root: Path) -> tuple[Path, Path]:
    """Create the minimum canonical/workspace tree with private Payments helpers."""
    canonical = root / "mycelix-finance" / "zomes"
    workspace = root / "mycelix-workspace" / "mycelix-finance" / "zomes"
    for zomes_root in (canonical, workspace):
        payments = zomes_root / "payments" / "coordinator" / "src" / "lib.rs"
        payments.parent.mkdir(parents=True, exist_ok=True)
        payments.write_text(
            "fn credit_sap(input: CreditSapInput) -> ExternResult<Record> { todo!() }\n",
            encoding="utf-8",
        )
    return canonical, workspace


def write_callers(zomes_root: Path) -> None:
    """Write the four known external dispatch shapes into one projection."""
    files = {
        "currency-mint/coordinator/src/lib.rs": 'let f = "credit_sap".into();\n',
        "bridge/coordinator/src/lib.rs": (
            'let a = FunctionName::from("credit_sap");\n'
            'let b = FunctionName::from(\n'
            '    "credit_sap"\n'
            ');\n'
        ),
        "staking/coordinator/src/lib.rs": 'let f = FunctionName::from("credit_sap");\n',
    }
    for relative, source in files.items():
        path = zomes_root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(source, encoding="utf-8")


class RawCreditCallerAuditTests(unittest.TestCase):
    def test_public_hdk_extern_raw_credit_is_detected(self) -> None:
        source = (
            "#[hdk_extern]\n"
            "pub fn credit_sap(input: CreditSapInput) -> ExternResult<Record> {\n"
            "    todo!()\n"
            "}\n"
        )
        self.assertTrue(has_public_raw_credit_abi(source))

    def test_private_internal_helper_is_not_a_public_raw_credit_abi(self) -> None:
        source = (
            "fn credit_sap(input: CreditSapInput) -> ExternResult<Record> {\n"
            "    todo!()\n"
            "}\n"
        )
        self.assertFalse(has_public_raw_credit_abi(source))

    def test_different_hdk_extern_does_not_trip_raw_credit_abi_guard(self) -> None:
        source = (
            "#[hdk_extern]\n"
            "pub fn debit_sap(input: DebitSapInput) -> ExternResult<Record> {\n"
            "    todo!()\n"
            "}\n"
        )
        self.assertFalse(has_public_raw_credit_abi(source))

    def test_multiline_function_name_dispatch_is_detected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "bridge" / "coordinator" / "src" / "lib.rs"
            source.parent.mkdir(parents=True)
            source.write_text(
                'let function = FunctionName::from(\n'
                '    "credit_sap"\n'
                ');\n',
                encoding="utf-8",
            )
            hits = scan(root)
            self.assertEqual(len(hits), 1)
            self.assertEqual(hits[0][0], "bridge/coordinator/src/lib.rs")
            self.assertEqual(hits[0][1], 1)
            self.assertIn("credit_sap", hits[0][2])

    def test_multiline_into_dispatch_is_detected_and_private_helper_is_ignored(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            external = root / "staking" / "coordinator" / "src" / "lib.rs"
            external.parent.mkdir(parents=True)
            external.write_text(
                'let function =\n'
                '    "credit_sap"\n'
                '        .into();\n',
                encoding="utf-8",
            )
            internal = root / "payments" / "coordinator" / "src" / "lib.rs"
            internal.parent.mkdir(parents=True)
            internal.write_text(
                'let function = FunctionName::from("credit_sap");\n',
                encoding="utf-8",
            )
            hits = scan(root)
            self.assertEqual(len(hits), 1)
            self.assertEqual(hits[0][0], "staking/coordinator/src/lib.rs")
            self.assertEqual(hits[0][1], 2)

    def test_non_coordinator_source_is_not_a_cross_zome_caller(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "bridge" / "integrity" / "src" / "lib.rs"
            source.parent.mkdir(parents=True)
            source.write_text(
                'let function = FunctionName::from("credit_sap");\n',
                encoding="utf-8",
            )
            self.assertEqual(scan(root), [])

    def test_whole_audit_fails_if_public_credit_abi_is_restored(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            seed_project(root)
            public_source = (
                "#[hdk_extern]\n"
                "pub fn credit_sap(input: CreditSapInput) -> ExternResult<Record> { todo!() }\n"
            )
            path = root / "mycelix-finance" / "zomes" / "payments" / "coordinator" / "src" / "lib.rs"
            path.write_text(public_source, encoding="utf-8")

            output = io.StringIO()
            with redirect_stdout(output):
                status = audit(root)

            self.assertEqual(status, 1)
            self.assertIn("remains exposed as a Holochain extern", output.getvalue())

    def test_whole_audit_fails_closed_on_the_four_known_callers(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            canonical, workspace = seed_project(root)
            write_callers(canonical)
            write_callers(workspace)

            output = io.StringIO()
            with redirect_stdout(output):
                status = audit(root)

            self.assertEqual(status, 1)
            self.assertIn("Found 4 call site(s) in each Finance projection.", output.getvalue())

    def test_whole_audit_rejects_projection_drift(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            canonical, _workspace = seed_project(root)
            source = canonical / "bridge" / "coordinator" / "src" / "lib.rs"
            source.parent.mkdir(parents=True, exist_ok=True)
            source.write_text(
                'let function = FunctionName::from("credit_sap");\n',
                encoding="utf-8",
            )

            output = io.StringIO()
            with redirect_stdout(output):
                status = audit(root)

            self.assertEqual(status, 1)
            self.assertIn("inventories differ", output.getvalue())

    def test_whole_audit_passes_only_when_abi_is_private_and_no_callers_exist(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            seed_project(root)

            output = io.StringIO()
            with redirect_stdout(output):
                status = audit(root)

            self.assertEqual(status, 0)
            self.assertIn("ABI is private", output.getvalue())


if __name__ == "__main__":
    unittest.main()
