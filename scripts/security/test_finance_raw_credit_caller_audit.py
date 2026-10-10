#!/usr/bin/env python3
# Copyright (C) 2024-2026 Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Regression tests for the fail-closed Finance raw-credit caller audit."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from finance_raw_credit_caller_audit import scan


class RawCreditCallerAuditTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
