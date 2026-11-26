# Craft Federation Agreement V1

- **Work item:** CRAFT-FEDERATION-AGREEMENT-001 (#6737)
- **Status:** Reference-core proposal; qualification pending
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## Purpose

This contract models negotiated cooperation among autonomous institutions. It answers one narrow question: has a particular institution agreed to rely on another institution for one exact, versioned purpose and scope?

It does not create a federation government, global identity registry, shared treasury, professional licensing authority or resource allocator.

## Directional agreement model

An agreement names its parties, exact terms digest, validity window, locally authorized decision references, purpose-bound scopes and directed interoperation edges.

An edge A→B means that A accepts the specified offer/framework from B for the exact scope. B does not thereby accept A. The agreement validator checks the explicit edge list; the evaluator does not traverse a graph. Therefore A→B and B→C cannot imply A→C.

For V1, every named party must consent to the exact agreement digest. A change to terms, scope, local decision reference or party list creates a new revision/digest and requires fresh consent.

## Cooperation scopes

Each scope pins:
- kind of cooperation;
- scope identifier, revision and digest;
- purpose;
- jurisdiction;
- minimum disclosure-profile reference.

Initial scope kinds include evidence-framework recognition, assessor-framework recognition, training exchange, project participation, evidence disclosure, mutual-aid coordination and shared-resource coordination.

These scope labels are not effects:
- Recognizing an assessor framework does not appoint an assessor or prove a particular assessment.
- Participating in a project does not grant local project authority.
- Coordinating a shared resource does not prove inventory, reserve the resource, transfer custody or authorize use.
- An evidence-disclosure profile does not itself authorize data disclosure; the data controller must authorize the specific release.

## Local sovereignty and consent

Each institution references its own adopted charter and local authorization decision. The host must authenticate the institution, charter adoption, decision issuer, signer, authority scope, revocation and freshness before treating the snapshot as trusted.

Each party's consent binds:
- its institution identity;
- the exact current terms digest;
- its exact local authority decision;
- acceptance time;
- optional effective withdrawal time.

Any missing party consent, changed digest or effective withdrawal prevents new interoperation. Suspension and termination are owner-supplied lifecycle states; the reference evaluator denies them.

## No transitive trust

No code path in the reference evaluator performs graph traversal or permission inheritance. Trust is a direct lookup on (from institution, to institution, exact scope). Shared membership of a federation is not sufficient.

A future federation directory may help discover possible agreements, but it must not become an authority source. The parties negotiate and authorize each direct edge themselves.

## Non-goals and ownership boundaries

- Identity authenticates institutional principals, signing keys and controllers.
- Each local governance owner authorizes its institution's participation.
- Work Evidence verifies attestations; professional authorities control assessor permissions.
- Resource owners control inventory, booking, allocation, custody and release.
- Praxis controls learner records and learning agreements.
- Manufacturing owns production and material-use events.
- Symtropy owns its physical-observation qualification contracts.
- Finance, issuers and payment providers own financial effects and settlement.
- Craft validates the agreement reference and the requested cooperation scope only.

No agreement result is a payment instruction, resource reservation, professional licence or authorization to start work.

## Qualification gate

Before production integration:
1. Run formatting, unit tests and Clippy against the exact commit.
2. Add property/fuzz tests for arbitrary institutional graphs and duplicate edges.
3. Freeze golden vectors and reproduce them in a second language.
4. Test withdrawal, suspension, expiry and revision races under partition/reconciliation.
5. Verify that the host authenticates all party and charter inputs.
6. Review data-disclosure profiles with the actual data controllers.
7. Test against bilateral negotiation and manually coordinated multilateral agreements in a prospective pilot.

Authored tests are not passing tests. Reference-core tests are not distributed-host qualification. Synthetic tests are not proof of legal enforceability or improved community outcomes.
