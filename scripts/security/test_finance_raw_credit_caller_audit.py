#!/usr/bin/env python3
# Copyright (C) 2024-2026 Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Regression tests for the fail-closed Finance raw-credit caller audit."""

from __future__ import annotations

import io
import re
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
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




def _rust_lexeme_end(source: str, index: int) -> int:
    """Return the end of a comment/string/char token, or index if none begins."""
    if source.startswith("//", index):
        newline = source.find("\n", index + 2)
        return len(source) if newline < 0 else newline + 1

    if source.startswith("/*", index):
        depth = 1
        cursor = index + 2
        while cursor < len(source) and depth:
            if source.startswith("/*", cursor):
                depth += 1
                cursor += 2
            elif source.startswith("*/", cursor):
                depth -= 1
                cursor += 2
            else:
                cursor += 1
        if depth:
            raise AssertionError("unterminated Rust block comment in function body")
        return cursor

    # Raw Rust string tokens: r"...", r##"..."##, br"...", br##"..."##.
    prefix = re.match(r"(?:br|cr|r)(?P<hashes>#{0,})\"", source[index:])
    if prefix is not None:
        hashes = prefix.group("hashes")
        opening_quote = index + prefix.end() - 1
        closing = source.find("\"" + hashes, opening_quote + 1)
        if closing < 0:
            raise AssertionError("unterminated Rust raw string in function body")
        return closing + 1 + len(hashes)

    # Ordinary (including byte/C-prefixed, which are skipped at their quote).
    if source[index] == '"':
        cursor = index + 1
        while cursor < len(source):
            if source[cursor] == "\\":
                cursor += 2
            elif source[cursor] == '"':
                return cursor + 1
            else:
                cursor += 1
        raise AssertionError("unterminated Rust string in function body")

    # A lifetime such as 'a is not a character literal; only skip a complete
    # one-scalar/escaped Rust character token.
    char_literal = re.match(r"'(?:\\.|[^'\\])'", source[index:])
    if char_literal is not None:
        return index + char_literal.end()
    return index


def rust_function_body(source: str, name: str) -> str:
    """Extract a Rust function body without matching function-shaped lexemes."""
    marker = re.compile(r"\bfn\s+" + re.escape(name) + r"\s*\(")
    start_match = None
    index = 0
    while index < len(source):
        lexeme_end = _rust_lexeme_end(source, index)
        if lexeme_end != index:
            index = lexeme_end
            continue
        start_match = marker.match(source, index)
        if start_match is not None:
            break
        index += 1

    if start_match is None:
        raise AssertionError(f"missing Rust function {name}")

    # Find the body opener after the parameter list and return/where clauses.
    # Track nested type delimiters so braces in array/const-generic types are not
    # mistaken for the function body.
    paren_depth = 1  # The marker consumes the opening parameter-list parenthesis.
    bracket_depth = 0
    angle_depth = 0
    index = start_match.end()
    opening = -1
    while index < len(source):
        lexeme_end = _rust_lexeme_end(source, index)
        if lexeme_end != index:
            index = lexeme_end
            continue
        char = source[index]
        if char == "(":
            paren_depth += 1
        elif char == ")":
            paren_depth -= 1
        elif char == "[":
            bracket_depth += 1
        elif char == "]":
            bracket_depth -= 1
        elif char == "<":
            angle_depth += 1
        elif char == ">" and angle_depth:
            angle_depth -= 1
        elif char == "{" and paren_depth == bracket_depth == angle_depth == 0:
            opening = index
            break
        index += 1

    if opening < 0:
        raise AssertionError(f"missing body for Rust function {name}")

    depth = 0
    index = opening
    while index < len(source):
        lexeme_end = _rust_lexeme_end(source, index)
        if lexeme_end != index:
            index = lexeme_end
            continue
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
        index += 1
    raise AssertionError(f"unterminated Rust function {name}")

class RawCreditCallerAuditTests(unittest.TestCase):
    def test_rust_function_body_ignores_braces_in_literals_and_nested_comments(self) -> None:
        source = r'''
fn disabled(_input: String) -> Result<(), String> {
    let ordinary = "brace } and opener {";
    let raw = r###"raw } { // not a comment"###;
    let escaped = "quoted \\\\ } text";
    /* outer close } /* nested opener { */ outer tail { */
    // line comment contains } and {
    if ordinary.len() + raw.len() + escaped.len() > 0 {
        return Err("disabled".into());
    }
}
fn later() { panic!("must not be part of disabled body"); }
'''
        body = rust_function_body(source, "disabled")
        self.assertIn("outer tail", body)
        self.assertIn('return Err("disabled".into())', body)
        self.assertNotIn("fn later()", body)


    def test_rust_function_body_skips_function_shaped_comments_and_strings(self) -> None:
        source = r'''
// fn disabled(_input: String) -> Result<(), String> { line decoy }
 /*
fn disabled(_input: String) -> Result<(), String> { block decoy }
*/
const DOC: &str = r#"fn disabled() { raw-string decoy }"#;
fn disabled(_input: String) -> Result<(), String> {
    let raw = r###"literal with braces { } and fn disabled() {"###;
    let _ = raw;
    Err("actual function body".into())
}
'''
        body = rust_function_body(source, "disabled")
        self.assertIn('Err("actual function body".into())', body)
        self.assertNotIn("line decoy", body)
        self.assertNotIn("block decoy", body)
        self.assertNotIn("raw-string decoy", body)

        # Braces in an array-length expression are part of the signature, not
        # the function body opener.
        signature = (
            "fn sized(_input: [u8; { 2 }]) -> Result<(), String> {\n"
            '    Err("body after signature expression".into())\n'
            "}\n"
        )
        signature_body = rust_function_body(signature, "sized")
        self.assertIn('Err("body after signature expression".into())', signature_body)


    def test_public_hdk_extern_raw_credit_is_detected(self) -> None:
        source = (
            "#[hdk_extern]\n"
            "pub fn credit_sap(input: CreditSapInput) -> ExternResult<Record> {\n"
            "    todo!()\n"
            "}\n"
        )
        self.assertTrue(has_public_raw_credit_abi(source))

    def test_interposed_outer_attribute_does_not_hide_public_extern(self) -> None:
        source = (
            "#[hdk_extern]\\n"
            "#[allow(clippy::too_many_arguments)]\\n"
            "pub fn credit_sap(input: CreditSapInput) -> ExternResult<Record> { todo!() }\\n"
        )
        self.assertTrue(has_public_raw_credit_abi(source))

    def test_comments_between_extern_attribute_and_function_are_supported(self) -> None:
        cases = (
            "#[hdk_extern]\\n// export boundary comment\\n"
            "#[allow(clippy::too_many_arguments)]\\npub fn credit_sap(input: CreditSapInput) -> ExternResult<Record> { todo!() }",
            "#[hdk_extern] /* export boundary comment */ pub fn credit_sap(input: CreditSapInput) -> ExternResult<Record> { todo!() }",
        )
        for source in cases:
            with self.subTest(source=source):
                self.assertTrue(has_public_raw_credit_abi(source))

    def test_quoted_literal_in_comment_is_flagged_for_review(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "bridge" / "coordinator" / "src" / "lib.rs"
            source.parent.mkdir(parents=True)
            source.write_text('// avoid calling "credit_sap" here\\n', encoding="utf-8")
            hits = scan(root)
            self.assertEqual(len(hits), 1)
            self.assertEqual(hits[0][1], 1)

    def test_unrelated_extern_does_not_hide_later_private_helper(self) -> None:
        source = (
            "#[hdk_extern]\\n"
            "pub fn debit_sap(input: DebitSapInput) -> ExternResult<Record> { todo!() }\\n"
            "fn credit_sap(input: CreditSapInput) -> ExternResult<Record> { todo!() }\\n"
        )
        self.assertFalse(has_public_raw_credit_abi(source))

    def test_direct_constructor_and_raw_string_literal_are_detected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            caller = root / "bridge" / "coordinator" / "src" / "lib.rs"
            caller.parent.mkdir(parents=True)
            caller.write_text(
                'let function = FunctionName::new(r#"credit_sap"#);',
                encoding="utf-8",
            )
            self.assertEqual(len(scan(root)), 1)

    def test_escaped_string_literal_dispatch_is_detected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            caller = root / "bridge" / "coordinator" / "src" / "lib.rs"
            caller.parent.mkdir(parents=True)
            caller.write_text(
                r'let function = FunctionName::from("credit\x5fsap");',
                encoding="utf-8",
            )
            hits = scan(root)
            self.assertEqual(len(hits), 1)
            self.assertIn('decoded static string "credit_sap"', hits[0][2])

    def test_unicode_escape_string_literal_dispatch_is_detected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            caller = root / "bridge" / "coordinator" / "src" / "lib.rs"
            caller.parent.mkdir(parents=True)
            caller.write_text(
                r'let function = FunctionName::from("credit\u{5f}sap");',
                encoding="utf-8",
            )
            hits = scan(root)
            self.assertEqual(len(hits), 1)
            self.assertIn('decoded static string "credit_sap"', hits[0][2])

    def test_concat_character_literals_are_detected(self) -> None:
        for source in (
            r"""let function = FunctionName::from(concat!('c', "redit", '_', "sap"));""",
            r"""let function = FunctionName::from(concat!('c', "redit", '\u{5f}', "sap"));""",
        ):
            with self.subTest(source=source), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                caller = root / "bridge" / "coordinator" / "src" / "lib.rs"
                caller.parent.mkdir(parents=True)
                caller.write_text(source, encoding="utf-8")
                hits = scan(root)
                self.assertEqual(len(hits), 1)
                self.assertIn('static function name "credit_sap"', hits[0][2])

    def test_compile_time_concat_dispatch_is_detected(self) -> None:
        for source in (
            'let function = FunctionName::from(concat!(\n    "credit_",\n    "sap"\n));\n',
            'let function = FunctionName::from(concat!(r#"credit_"#, /* ) */ r#"sap"#));\n',
        ):
            with self.subTest(source=source), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                caller = root / "bridge" / "coordinator" / "src" / "lib.rs"
                caller.parent.mkdir(parents=True)
                caller.write_text(source, encoding="utf-8")
                hits = scan(root)
                self.assertEqual(len(hits), 1)
                self.assertIn('static function name "credit_sap"', hits[0][2])

    def test_alternate_dispatch_constructors_are_detected(self) -> None:
        for source in (
            'FunctionName::try_from("credit_sap")',
            '"credit_sap".to_string()',
        ):
            with self.subTest(source=source), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                caller = root / "bridge" / "coordinator" / "src" / "lib.rs"
                caller.parent.mkdir(parents=True)
                caller.write_text(source, encoding="utf-8")
                self.assertEqual(len(scan(root)), 1)

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
            self.assertEqual(hits[0][1], 2)
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

    def test_ci_audit_job_covers_workspace_and_its_own_sources(self) -> None:
        repository_root = Path(__file__).resolve().parents[2]
        workflow_path = repository_root / ".github" / "workflows" / "ci.yml"
        workflow = workflow_path.read_text(encoding="utf-8")

        self.assertIn(
            "finance_raw_credit_audit: ${{ steps.filter.outputs.finance_raw_credit_audit }}",
            workflow,
        )
        filter_start = workflow.index("            finance_raw_credit_audit:")
        filter_end = workflow.index("\n            governance:", filter_start)
        filter_block = workflow[filter_start:filter_end]
        for required_path in (
            "'mycelix-finance/**'",
            "'mycelix-workspace/mycelix-finance/**'",
            "'scripts/security/finance_raw_credit_caller_audit.py'",
            "'scripts/security/test_finance_raw_credit_caller_audit.py'",
            "'.github/workflows/ci.yml'",
        ):
            with self.subTest(required_path=required_path):
                self.assertIn(required_path, filter_block)

        job_start = workflow.index("  finance-raw-credit-caller-audit:")
        job_end = workflow.index("\n  test-finance:", job_start)
        job_block = workflow[job_start:job_end]
        self.assertIn(
            "needs.changes.outputs.finance_raw_credit_audit == 'true'",
            job_block,
        )
        self.assertIn("github.event_name == 'workflow_dispatch'", job_block)
        self.assertIn(
            "python3 -m unittest discover -s scripts/security -p 'test_finance_raw_credit_caller_audit.py' -v",
            job_block,
        )
        ci_pass_start = workflow.index("  ci-pass:")
        ci_pass_end = workflow.index("\n    runs-on:", ci_pass_start)
        ci_pass = workflow[ci_pass_start:ci_pass_end]
        self.assertIn("- changes", ci_pass)
        self.assertIn("- finance-raw-credit-caller-audit", ci_pass)
        check_results_start = workflow.index("      - name: Check results", ci_pass_start)
        check_results = workflow[check_results_start:]
        failure_loop_start = workflow.index("for result in ", ci_pass_start)
        failure_loop = workflow[failure_loop_start:]
        self.assertIn(
            '"${{ needs.finance-raw-credit-caller-audit.result }}"',
            failure_loop,
        )
        self.assertIn('if [ "$result" = "failure" ]; then exit 1; fi', failure_loop)
        self.assertIn(
            'if [ "${{ needs.changes.result }}" != "success" ]; then',
            check_results,
        )
        self.assertIn('needs.changes.outputs.finance_raw_credit_audit', check_results)
        self.assertIn('|| [ "${{ github.event_name }}" = "workflow_dispatch" ]; then', check_results)
        self.assertIn(
            'if [ "${{ needs.finance-raw-credit-caller-audit.result }}" != "success" ]; then',
            check_results,
        )

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
            self.assertIn("Found 4 matching literal(s) in each Finance projection.", output.getvalue())

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

    def test_whole_audit_rejects_non_caller_projection_content_drift(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _canonical, workspace = seed_project(root)
            payments = workspace / "payments" / "coordinator" / "src" / "lib.rs"
            payments.write_text(
                payments.read_text(encoding="utf-8") + "// workspace-only drift\n",
                encoding="utf-8",
            )

            output = io.StringIO()
            with redirect_stdout(output):
                status = audit(root)

            self.assertEqual(status, 1)
            self.assertIn(
                "canonical and workspace Finance zome file projections differ",
                output.getvalue(),
            )
            self.assertIn(
                "payments/coordinator/src/lib.rs: content differs",
                output.getvalue(),
            )

    def test_whole_audit_rejects_symlinks_before_source_reads(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _canonical, workspace = seed_project(root)
            linked_source = workspace / "bridge" / "coordinator" / "src" / "lib.rs"
            linked_source.parent.mkdir(parents=True, exist_ok=True)
            linked_source.symlink_to(root / "missing-source.rs")

            errors = io.StringIO()
            with redirect_stderr(errors):
                status = audit(root)

            self.assertEqual(status, 2)
            self.assertIn("unexpected symlink in Finance zome projection", errors.getvalue())

    def test_whole_audit_rejects_symlinked_projection_root(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            canonical, workspace = seed_project(root)
            saved_workspace = root / "workspace-zomes-original"
            workspace.rename(saved_workspace)
            workspace.symlink_to(canonical, target_is_directory=True)

            errors = io.StringIO()
            with redirect_stderr(errors):
                status = audit(root)

            self.assertEqual(status, 2)
            self.assertIn(
                "unexpected symlink used as Finance zome projection root",
                errors.getvalue(),
            )

    def test_whole_audit_passes_only_when_abi_is_private_and_no_callers_exist(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            seed_project(root)

            output = io.StringIO()
            with redirect_stdout(output):
                status = audit(root)

            self.assertEqual(status, 0)
            self.assertIn("ABI is private", output.getvalue())
            self.assertIn(
                "no direct, escaped/Unicode-escaped, or statically concatenated raw-credit function-name reference remains",
                output.getvalue(),
            )


    def test_legacy_external_sap_effects_are_source_level_fail_closed(self) -> None:
        repository_root = Path(__file__).resolve().parents[2]
        cases = (
            (
                "mycelix-finance/zomes/currency-mint/coordinator/src/lib.rs",
                "mint_genesis_sap",
                "Thermodynamic genesis issuance is disabled",
            ),
            (
                "mycelix-finance/zomes/bridge/coordinator/src/lib.rs",
                "deposit_collateral",
                "Legacy collateral SAP issuance is disabled",
            ),
            (
                "mycelix-finance/zomes/bridge/coordinator/src/lib.rs",
                "deposit_fiat",
                "Fiat deposit intake is disabled",
            ),
            (
                "mycelix-finance/zomes/bridge/coordinator/src/lib.rs",
                "verify_fiat_deposit",
                "Fiat-deposit verification and SAP issuance are disabled",
            ),
            (
                "mycelix-finance/zomes/bridge/coordinator/src/lib.rs",
                "confirm_deposit",
                "Legacy collateral confirmation is disabled",
            ),
            (
                "mycelix-finance/zomes/staking/coordinator/src/lib.rs",
                "slash_stake",
                "Staking slashing is disabled",
            ),
        )
        forbidden_effects = (
            "create_entry(",
            "create_link(",
            "update_entry(",
            "delete_entry(",
            "call(",
        )
        for relative, function_name, message in cases:
            with self.subTest(function=function_name):
                source = (repository_root / relative).read_text(encoding="utf-8")
                body = rust_function_body(source, function_name)
                self.assertTrue(body.strip().startswith("Err(wasm_error!("))
                self.assertIn(message, body)
                for effect in forbidden_effects:
                    self.assertNotIn(effect, body)

        staking = (
            repository_root / "mycelix-finance/zomes/staking/coordinator/src/lib.rs"
        ).read_text(encoding="utf-8")
        return_body = rust_function_body(staking, "return_stake_sap")
        self.assertIn("if amount == 0", return_body)
        self.assertIn("return Ok(())", return_body)
        self.assertIn("Non-zero staking returns are disabled", return_body)
        self.assertNotIn("call(", return_body)

    def test_staking_withdrawal_requires_return_before_terminal_mutation(self) -> None:
        repository_root = Path(__file__).resolve().parents[2]
        source = (
            repository_root / "mycelix-finance/zomes/staking/coordinator/src/lib.rs"
        ).read_text(encoding="utf-8")
        body = rust_function_body(source, "withdraw_stake")
        return_at = body.find("return_stake_sap(")
        terminal_mutation_at = body.find("let action_hash = update_entry(")
        self.assertGreaterEqual(return_at, 0)
        self.assertGreater(terminal_mutation_at, return_at)
        self.assertIn(")?;", body[return_at:terminal_mutation_at])


if __name__ == "__main__":
    unittest.main()
