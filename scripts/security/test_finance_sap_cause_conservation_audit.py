import unittest

from scripts.security.finance_sap_cause_conservation_audit import (
    _body,
    extract_functions,
    mask_rust_noncode,
)


class AuditHardeningTests(unittest.TestCase):
    def test_comment_decoy_is_masked(self):
        source = r"""
        fn credit_sap() {
            // input.justified_by.is_none()
            return;
        }
        """
        masked = mask_rust_noncode(source)
        fn = extract_functions(source)["credit_sap"]
        self.assertNotIn("input.justified_by.is_none()", _body(masked, fn))

    def test_string_decoy_is_masked(self):
        source = r'''
        fn credit_sap() {
            let _ = "input.justified_by.is_none()";
        }
        '''
        masked = mask_rust_noncode(source)
        fn = extract_functions(source)["credit_sap"]
        self.assertNotIn("input.justified_by.is_none()", _body(masked, fn))

    def test_raw_string_braces_do_not_break_function_extraction(self):
        source = r'''
        fn find_sap_balance_record() {
            let _ = r###" } } follow_update_chain() { "###;
            follow_update_chain_strict();
        }
        fn next() {}
        '''
        functions = extract_functions(source)
        self.assertIn("find_sap_balance_record", functions)
        self.assertIn("next", functions)
        self.assertLess(
            functions["find_sap_balance_record"].body_end,
            len(source),
        )

    def test_unrelated_strict_lookup_decoy_is_not_authoritative(self):
        source = r'''
        fn helper() {
            follow_update_chain_strict();
        }
        fn find_sap_balance_record() {
            follow_update_chain();
        }
        '''
        masked = mask_rust_noncode(source)
        fn = extract_functions(source)["find_sap_balance_record"]
        body = _body(masked, fn)
        self.assertNotIn("follow_update_chain_strict()", body)
        self.assertIn("follow_update_chain()", body)

    def test_capacity_is_not_exact_conservation(self):
        source = r'''
        fn validate_update_sap_balance() {
            if debited < credited_amount {
                reject();
            }
        }
        '''
        masked = mask_rust_noncode(source)
        self.assertIn("debited < credited_amount", masked)
        self.assertNotIn("debited == credited_amount", masked)


if __name__ == "__main__":
    unittest.main()
