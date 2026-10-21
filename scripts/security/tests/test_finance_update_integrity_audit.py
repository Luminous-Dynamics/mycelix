"""Mutation-resistant contract tests for the Finance predecessor audit."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import unittest


SECURITY_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = SECURITY_DIR / "finance_update_integrity_audit.py"
SPEC = spec_from_file_location("finance_update_integrity_audit", SCRIPT_PATH)
AUDIT = module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(AUDIT)


GOOD_SOURCE = """
fn validate_update_payment(
    action: Update,
    payment: Payment,
) -> ExternResult<ValidateCallbackResult> {
    let original = must_get_valid_record(action.original_action_address.clone())?;
    let predecessor = original.entry().to_app_option::<Payment>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("{e:?}"))))?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("wrong predecessor type".into())))?;
    if predecessor.id != payment.id {
        return Ok(ValidateCallbackResult::Invalid("identity changed".into()));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op {
        OpEntry::UpdateEntry { action, entry, .. } => match entry {
            EntryTypes::Payment(payment) => validate_update_payment(action, payment)?,
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}
"""


class FinancePredecessorAuditTests(unittest.TestCase):
    def audit(self, source):
        errors, count = AUDIT.audit_source("fixture.rs", source)
        self.assertEqual(count, 1, "fixture must contain exactly one update validator")
        return errors

    def test_exact_predecessor_dependency_and_update_dispatch_pass(self):
        self.assertEqual(self.audit(GOOD_SOURCE), [])

    def test_missing_predecessor_is_rejected(self):
        changed = GOOD_SOURCE.replace(
            "let original = must_get_valid_record(action.original_action_address.clone())?;",
            "let original = payment.clone();",
        )
        errors = self.audit(changed)
        self.assertTrue(any("missing must_get_valid_record" in error for error in errors))

    def test_wrong_predecessor_reference_is_rejected(self):
        changed = GOOD_SOURCE.replace(
            "action.original_action_address.clone()",
            "action.some_other_hash.clone()",
        )
        errors = self.audit(changed)
        self.assertTrue(any("not bound to action.original_action_address" in error for error in errors))

    def test_swallowed_host_dependency_is_rejected(self):
        changed = GOOD_SOURCE.replace(
            "let original = must_get_valid_record(action.original_action_address.clone())?;",
            "if let Ok(original) = must_get_valid_record(action.original_action_address.clone()) {",
        )
        errors = self.audit(changed)
        self.assertTrue(any("swallowed must_get_valid_record" in error for error in errors))
        self.assertTrue(any("failure is not propagated with ?" in error for error in errors))

    def test_unpropagated_host_dependency_is_rejected(self):
        changed = GOOD_SOURCE.replace(
            "let original = must_get_valid_record(action.original_action_address.clone())?;",
            "let original = must_get_valid_record(action.original_action_address.clone());",
        )
        errors = self.audit(changed)
        self.assertTrue(any("failure is not propagated with ?" in error for error in errors))

    def test_direct_valid_update_dispatch_arm_is_rejected(self):
        changed = GOOD_SOURCE.replace(
            "EntryTypes::Payment(payment) => validate_update_payment(action, payment)?,",
            "EntryTypes::Payment(payment) => Ok(ValidateCallbackResult::Valid),",
        )
        errors = self.audit(changed)
        self.assertTrue(any("direct Valid in UpdateEntry arm" in error for error in errors))

    def test_update_dispatch_to_non_update_validator_is_rejected(self):
        changed = GOOD_SOURCE.replace(
            "EntryTypes::Payment(payment) => validate_update_payment(action, payment)?,",
            "EntryTypes::Payment(payment) => validate_payment(action, payment)?,",
        )
        errors = self.audit(changed)
        self.assertTrue(any("delegates to non-update validator validate_payment" in error for error in errors))

    def test_comments_and_strings_do_not_manufacture_a_validator(self):
        source = '''
// fn validate_update_fake(action: Update) { Ok(ValidateCallbackResult::Valid) }
const EXAMPLE: &str = "fn validate_update_fake(action: Update) { }";
fn validate_create_payment(action: EntryCreationAction) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}
'''
        errors, count = AUDIT.audit_source("fixture.rs", source)
        self.assertEqual(errors, [])
        self.assertEqual(count, 0)

    def test_missing_finance_roots_fail_closed(self):
        from tempfile import TemporaryDirectory

        with TemporaryDirectory() as directory:
            errors, files, validators = AUDIT.audit_tree(Path(directory))
        self.assertTrue(any("Finance integrity root is missing" in error for error in errors))
        self.assertEqual(files, 0)
        self.assertEqual(validators, 0)


if __name__ == "__main__":
    unittest.main()
