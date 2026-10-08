# Censoring Classification Anchor Governance Research v1

Status: research-only trust-root separation and deterministic campaign.

## Purpose

#4817 establishes object identity continuity relative to a supplied anchor. This follow-on separates the supplied anchor from the mutable subject fixture by introducing:

    externally pinned trust root
      -> current anchor manifest
      -> prior manifest commitment
      -> exact subject-fixture Git blob
      -> object identifier + type + semantic commitment

The manifest is intentionally not treated as a signature authority. The verifier receives an externally supplied root digest and rejects a root or current manifest that does not match it.

## What is now checked

The governance verifier distinguishes:

    fixed external root
        !=
    mutable subject fixture
        !=
    mutable current manifest

It checks:

- exact current manifest commitment under the pinned trust root;
- exact manifest version range and current epoch;
- previous-manifest linkage;
- exact subject fixture Git blob identity;
- exact policy Git blob identity;
- claim-scope anchor identity;
- object type + commitment identity for anchored IDs;
- registration not later than the subject classification epoch;
- active/revoked state;
- manifest freshness/expiry;
- representation invariance;
- rollback and fast-forward rejection;
- manifest/authority substitution;
- mixed-snapshot rejection;
- policy/scope substitution;
- previous-link corruption.

## Fixed versus semantic-liveness modes

fixed-pin cases keep the external root digest constant. They model an attacker changing the subject or manifest while the independently established root remains unchanged.

semantic-liveness cases deliberately derive a new root digest after a mutation. These do not demonstrate trust-root security; they demonstrate that the verifier still rejects an internally inconsistent or stale anchor state even when the mutated root is presented as externally pinned.

This distinction prevents a recomputed root from being mistaken for independent custody.

## Corpus discipline

The existing classification corpus remains exactly 52 fixed rows / 168 generated rows. Anchor governance uses a separate 21-case campaign so it does not inflate the classification count.

The fixed classification IDs are now required to be unique. This closes a composition defect where policy-liveness lookup by case ID could silently select the wrong duplicate row.

## Important ceiling

The current root digest is supplied by the workflow invocation. Because this workflow remains in the same repository branch, a hostile PR can rewrite the workflow and change that pin. Therefore this is a repository-level trust-separation theorem, not yet independent hosted authority.

The next ceiling remains a protected default-branch S0/S1/S2 mechanism, as described by #1146: trusted discovery, unprivileged exact-subject execution, and trusted receipt verification.

## Claim ceiling

Even with the trust-root separation, this evidence establishes only:

    this exact subject object
      matches
    this exact independently pinned research manifest lineage

It does not establish issuer honesty, source truth, causal correctness, trustworthy wall-clock time, completeness, or external-world truth.
