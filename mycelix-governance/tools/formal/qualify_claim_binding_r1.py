#!/usr/bin/env python3
"""Repair MYC-CONST-003B4 qualification mutation isolation without changing semantics.

The original exact-head qualifier at 7a654d6c... correctly failed because its
`disable-restored-binding-census` mutant survived.  The semantic invariant has
defense in depth: the direct missing-binding guard and the final binding census
both reject the same malformed restored state.  The old mutant bypassed only the
first guard, so the second guard still caught it.

This wrapper leaves the canonical semantic subject and all other controls
unchanged.  It replaces only that mutant so the mutated program bypasses the
whole missing-binding census path for that entry, allowing the existing recovery
test to demonstrate that the census is mutation-sensitive.
"""

from pathlib import Path

import qualify_claim_binding as base

THIS_SCRIPT = Path(__file__).resolve()
if THIS_SCRIPT not in base.INPUTS:
    base.INPUTS.append(THIS_SCRIPT)

mutation = base.RUST_MUTATIONS["disable-restored-binding-census"]
mutation["before"] = """                if !observed_finality_ids.insert(evidence_id.as_str()) {
                    return Err(ClaimLifecycleError::InvariantViolation);
                }
                let Some(binding) = self.temporal.finality_binding(evidence_id) else {
                    return Err(ClaimLifecycleError::InvariantViolation);
                };
"""
mutation["after"] = """                let Some(binding) = self.temporal.finality_binding(evidence_id) else {
                    continue;
                };
                if !observed_finality_ids.insert(evidence_id.as_str()) {
                    return Err(ClaimLifecycleError::InvariantViolation);
                }
"""

if __name__ == "__main__":
    raise SystemExit(base.main())
