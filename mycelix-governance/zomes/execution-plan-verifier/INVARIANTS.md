# Governance Execution Plan Verifier v0.1

Status: **normative provider contract**

This provider proves immutable executable-plan provenance. It does not mint institutional authority.

## Source of plan identity

The only v0.1 plan source is the exact historical Proposal action referenced by the currently qualified `ProposalAuthorityBinding.proposal_action_hash`.

That action hash is immutable. The provider fetches that exact action directly; it does not:

- call `proposals::get_proposal` for executable bytes;
- follow a latest Proposal update;
- use `Timelock.actions`;
- use `Timelock.status`; or
- select a version by timestamp, DHT arrival order, or update ordering.

The historical action may be a Proposal `Create` or author-authored Draft `Update`. This permits a future #66 repair to support legitimate Draft editing without changing the plan-provider protocol.

## Required provenance checks

The historical record must:

- be exactly the action hash named by the authority binding;
- be a Proposal Create or Update action;
- decode as the same proposal ID named by the authority context;
- retain Draft status at the point where content is frozen into institutional context;
- have a non-zero Proposal version;
- name the same proposal author as the authority binding;
- be actually authored by that same Holochain agent; and
- contain valid bounded JSON action bytes.

The actual Holochain action author is checked independently of the Proposal `author` string. A third-party update that merely copies the victim's DID cannot become a plan source.

## Exact action identity

The provider recomputes the existing governance execution digest profile over the exact historical action bytes:

`mycelix-governance-execution-authority-v1-blake3-exact-json`

The digest must equal `ProposalAuthorityBinding.context.actions_digest` and the binding must name the exact profile.

Whitespace or any other byte change therefore creates a different plan.

## Stable plan reference

`plan_ref` commits:

- exact historical Proposal action hash;
- exact action-digest profile; and
- exact action digest.

The provider verification reference separately names the exact proposal-authority binding action and exact historical proposal action.

## Authority separation

A positive plan receipt means only:

> These exact executable bytes are the immutable Proposal version referenced by the currently qualified proposal-authority binding.

It does **not** mean:

- the proposal passed a vote;
- the named institution/rulebook is legitimate;
- the constitution is current;
- a signing committee authorized execution;
- an executor is designated; or
- external effects are safe.

Those facts belong to execution preflight, threshold authority, executor designation, and effect-safety providers composed by PR #65.

## Legacy Proposal projection

Issue #66 tracks the separate mutable Proposal lifecycle defect. This provider deliberately does not solve that projection problem by choosing a latest record.

`proposal_authority.get_verified_proposal_authority_context` remains an upstream admission dependency. The later execution preflight independently verifies constitutional/tally/action authority, so this provider must not be interpreted as legitimizing the legacy Proposal status plane.

## Packaging

This provider is intentionally compiled but not packaged into the binding governance DNA in this tranche. The lifecycle verifier remains unprovisioned until the full provider stack qualifies together.
