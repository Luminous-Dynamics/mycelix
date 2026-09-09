# Justice Adjudication Evidence v0.1

This zome adds append-only, individually authored evidence for Mycelix Justice arbitration.

It exists because the legacy `Arbitration.arbitrators[].accepted/recused` and `Decision.votes[]` fields are aggregate convenience state. They are not sufficient downstream authority: one entry author can represent multiple actors inside those aggregates.

## v0.1 evidence

### Arbitrator participation

`ArbitratorParticipationAttestationV1` binds:

- one exact `Arbitration` action hash;
- the exact `Arbitration.id` decoded from that action;
- the attesting arbitrator DID;
- either acceptance or recusal.

The represented DID must equal the committing agent and must already be selected in the exact referenced Arbitration.

### Decision vote

`DecisionVoteAttestationV1` binds:

- one exact `Decision` action hash;
- the exact `Decision.id` decoded from that action;
- one exact `Arbitration` action hash and ID;
- the exact ActionHash of an individually authored `Accepted` participation attestation;
- the attesting arbitrator DID;
- that arbitrator's vote choice.

Integrity loads the exact acceptance attestation and requires it to name the same arbitrator, the same exact Arbitration action/ID, and `ParticipationDispositionV1::Accepted`. Another arbitrator's acceptance, an acceptance for another Arbitration action, or a recusal cannot establish the vote's participation basis.

Because the vote names the exact Decision **action**, the attestation concerns that immutable Decision artifact, including its exact outcome/remedies—not merely a free-floating winner label. A different Decision action requires a different attestation even when a human-readable ID is reused.

The v0.1 authority chain is therefore:

```text
exact Arbitration action
    -> individually authored Accepted participation action
        -> exact Decision action
            -> individually authored vote action
```

## Append-only and conflict preserving

Attestation entries cannot be updated or deleted.

This tranche deliberately does **not** define `latest wins` or otherwise collapse multiple attestations from the same arbitrator. A later Justice-owned reducer/verifier must:

- require the exact policy-defined evidence set;
- detect duplicate/conflicting participation attestations;
- detect duplicate/conflicting vote attestations;
- treat a later/conflicting recusal as explicit ambiguity rather than erasing earlier evidence;
- fail closed on ambiguity unless an explicit versioned supersession policy exists;
- never substitute the legacy aggregate fields when authenticated evidence is incomplete.

Links are discovery indexes only. The attestation entry plus its exact referenced action hashes and DHT authorship are the evidence.

## Authority boundary

This zome proves only that an individually authored claim is attributable to an exact selected arbitrator and exact referenced Justice records, and that a vote carries an exact individually authored acceptance basis.

It does **not** prove:

- that no conflicting/later participation evidence exists;
- quorum or decision-rule satisfaction;
- that the Decision remedy is executable;
- appeal finality;
- absence of appeals;
- Finance payment/refund/settlement truth;
- Business closure.

In particular, an empty local appeal-link query must never be promoted into global `no appeal` truth in an eventually consistent DHT.

The intended sequence is:

```text
exact Arbitration
    -> individually authored participation evidence
        -> exact accepted-participation basis
            -> exact Decision
                -> individually authored vote evidence
                    -> Justice-owned conflict-aware reduction
                    -> authenticated appeal/finality evidence
                    -> pure justice-resolution-verifier
                    -> later Justice -> Finance/Business bridge
```

## Legacy compatibility

The existing arbitration zome remains unchanged in v0.1 for compatibility. Its mutable participation flags, embedded Decision votes, mutable Appeal status, and `Decision.finalized` flag must not be used as sufficient authority for downstream execution.
