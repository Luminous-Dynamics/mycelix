from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from capital_to_commons_v1 import (  # noqa: E402
    PROFILE_VERSION,
    TransitionError,
    canonical_bytes,
    qualify,
    sha256_hex,
)


def profile():
    return {
        "profile_version": PROFILE_VERSION,
        "project_id": "fiber:jhb:test-001",
        "unit": "ZAR-cent",
        "max_amount_units": 1_000_000_000,
        "initial_principal_units": 100_000_000,
        "preferred_return_cap_units": 20_000_000,
        "max_qualified_new_capital_units": 30_000_000,
        "max_recoverable_lifecycle_units": 10_000_000,
        "required_reserve_units": 5_000_000,
        "max_events": 100,
    }


def history(*specs, p=None):
    p = copy.deepcopy(p or profile())
    out = []
    psha = sha256_hex(p)
    for seq, spec in enumerate(specs):
        kind, amount, *rest = spec
        suffix = str(seq)
        counterparty = rest[0] if rest else None
        event = {
            "seq": seq,
            "event_id": f"evt-{suffix}",
            "project_id": p["project_id"],
            "profile_sha256": psha,
            "prev_event_sha256": None if seq == 0 else sha256_hex(out[-1]),
            "kind": kind,
            "amount_units": amount,
            "authority_ref": f"authority:{suffix}",
            "evidence_ref": f"evidence:{suffix}",
            "counterparty_ref": counterparty,
        }
        out.append(event)
    return out


def positive_events(p=None):
    return history(
        ("InitialCapitalContribution", 100_000_000),
        ("ReserveFunding", 5_000_000),
        ("InvestorDistribution", 40_000_000),
        ("RefinanceReplaceClaim", 0, "investor:b"),
        ("GrantOrSubsidy", 7_000_000, "public:grantor"),
        ("QualifiedNewCapital", 10_000_000, "investor:c"),
        ("ApprovedRecoverableLifecycleCost", 3_000_000, "contractor:lifecycle"),
        ("InvestorDistribution", 20_000_000, "investor:b"),
        p=p,
    )


class TransitionTests(unittest.TestCase):
    def test_positive_reconstruction(self):
        receipt = qualify(profile(), positive_events()).receipt()
        self.assertEqual(receipt["created_entitlement_units"], 133_000_000)
        self.assertEqual(receipt["counted_investor_distributions_units"], 60_000_000)
        self.assertEqual(receipt["remaining_claim_units"], 73_000_000)
        self.assertEqual(receipt["grant_or_subsidy_units"], 7_000_000)
        self.assertEqual(receipt["financial_state"], "CLAIM_ACTIVE")
        self.assertFalse(receipt["legal_transition_complete"])
        self.assertFalse(receipt["handback_accepted"])
        self.assertEqual(len(receipt["event_chain_tip_sha256"]), 64)

    def test_deterministic_receipt(self):
        a = qualify(profile(), positive_events()).receipt()
        b = qualify(copy.deepcopy(profile()), copy.deepcopy(positive_events())).receipt()
        self.assertEqual(canonical_bytes(a), canonical_bytes(b))
        self.assertEqual(sha256_hex(a), sha256_hex(b))

    def test_grant_does_not_increase_claim(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("GrantOrSubsidy", 50_000_000),
        )
        receipt = qualify(profile(), events).receipt()
        self.assertEqual(receipt["remaining_claim_units"], 120_000_000)
        self.assertEqual(receipt["grant_or_subsidy_units"], 50_000_000)

    def test_refinance_cannot_reset_claim(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("ReserveFunding", 5_000_000),
            ("InvestorDistribution", 50_000_000),
            ("RefinanceReplaceClaim", 0),
        )
        receipt = qualify(profile(), events).receipt()
        self.assertEqual(receipt["remaining_claim_units"], 70_000_000)
        self.assertEqual(receipt["retired_claim_units"], 50_000_000)

    def test_refinance_cannot_carry_amount(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("RefinanceReplaceClaim", 10_000_000),
        )
        with self.assertRaisesRegex(TransitionError, "amount_units=0"):
            qualify(profile(), events)

    def test_operator_change_cannot_reset_claim(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("OperatorChange", 0),
        )
        self.assertEqual(qualify(profile(), events).receipt()["remaining_claim_units"], 120_000_000)

    def test_distribution_requires_reserve(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("InvestorDistribution", 1),
        )
        with self.assertRaisesRegex(TransitionError, "required reserve is underfunded"):
            qualify(profile(), events)

    def test_reserve_draw_can_block_later_distribution(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("ReserveFunding", 5_000_000),
            ("ReserveDraw", 1_000_000),
            ("InvestorDistribution", 1_000_000),
        )
        with self.assertRaisesRegex(TransitionError, "required reserve is underfunded"):
            qualify(profile(), events)

    def test_distribution_cannot_underflow_claim(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("ReserveFunding", 5_000_000),
            ("InvestorDistribution", 120_000_001),
        )
        with self.assertRaisesRegex(TransitionError, "underflow claim"):
            qualify(profile(), events)

    def test_qualified_new_capital_is_bounded(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("QualifiedNewCapital", 30_000_000),
            ("QualifiedNewCapital", 1),
        )
        with self.assertRaisesRegex(TransitionError, "new capital cap exceeded"):
            qualify(profile(), events)

    def test_lifecycle_additions_are_bounded(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("ApprovedRecoverableLifecycleCost", 10_000_000),
            ("ApprovedRecoverableLifecycleCost", 1),
        )
        with self.assertRaisesRegex(TransitionError, "lifecycle cap exceeded"):
            qualify(profile(), events)

    def test_unknown_fee_class_fails_closed(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("ManagementFee", 1_000_000),
        )
        with self.assertRaisesRegex(TransitionError, "unsupported kind"):
            qualify(profile(), events)

    def test_broken_sequence_fails_closed(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("ReserveFunding", 5_000_000),
        )
        events[1]["seq"] = 2
        with self.assertRaisesRegex(TransitionError, "contiguous sequence"):
            qualify(profile(), events)

    def test_duplicate_event_id_fails_closed(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("ReserveFunding", 5_000_000),
        )
        events[1]["event_id"] = events[0]["event_id"]
        events[1]["prev_event_sha256"] = sha256_hex(events[0])
        with self.assertRaisesRegex(TransitionError, "duplicate"):
            qualify(profile(), events)

    def test_project_substitution_fails_closed(self):
        events = history(("InitialCapitalContribution", 100_000_000))
        events[0]["project_id"] = "fiber:other"
        with self.assertRaisesRegex(TransitionError, "project substitution"):
            qualify(profile(), events)

    def test_profile_substitution_fails_closed(self):
        events = history(("InitialCapitalContribution", 100_000_000))
        events[0]["profile_sha256"] = "0" * 64
        with self.assertRaisesRegex(TransitionError, "profile substitution"):
            qualify(profile(), events)

    def test_event_reorder_or_mutation_breaks_chain(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("ReserveFunding", 5_000_000),
            ("TransitionCheckpoint", 0),
        )
        events[1]["amount_units"] = 4_999_999
        with self.assertRaisesRegex(TransitionError, "broken event chain"):
            qualify(profile(), events)

    def test_initial_contribution_must_match_profile(self):
        events = history(("InitialCapitalContribution", 99_999_999))
        with self.assertRaisesRegex(TransitionError, "must equal frozen profile principal"):
            qualify(profile(), events)

    def test_zero_claim_is_financial_not_legal_completion(self):
        events = history(
            ("InitialCapitalContribution", 100_000_000),
            ("ReserveFunding", 5_000_000),
            ("InvestorDistribution", 120_000_000),
        )
        receipt = qualify(profile(), events).receipt()
        self.assertEqual(receipt["remaining_claim_units"], 0)
        self.assertEqual(receipt["financial_state"], "RETURN_ENVELOPE_SATISFIED")
        self.assertFalse(receipt["legal_transition_complete"])
        self.assertFalse(receipt["handback_accepted"])

    def test_unknown_profile_key_fails_closed(self):
        p = profile()
        p["roi"] = 42
        with self.assertRaisesRegex(TransitionError, "unknown=.*roi"):
            qualify(p, positive_events())

    def test_max_possible_envelope_must_fit_authority_bound(self):
        p = profile()
        p["max_amount_units"] = 120_000_000
        with self.assertRaisesRegex(TransitionError, "maximum possible return envelope"):
            qualify(p, positive_events(p))


if __name__ == "__main__":
    unittest.main()
