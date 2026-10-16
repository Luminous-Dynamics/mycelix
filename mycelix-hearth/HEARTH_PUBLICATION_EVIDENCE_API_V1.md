# Hearth Publication Evidence API V1

**Scope:** Holochain 0.6 hearth_bridge coordinator endpoint.
**Status:** implementation and tests authored; no Rust/WASM or Sweettest pass claimed yet.
**Runtime consumer:** not yet wired into the Leptos UI.

## Why this API exists

A successful Holochain zome result is evidence of the returned operation/source-chain outcome. It is not proof that DHT publication has reached a receipt threshold, and a receipt threshold is not proof that every peer currently has the data.

The published HDK 0.6.0 source exposes the get_validation_receipts(GetValidationReceiptsInput) function returning ExternResult<Vec<ValidationReceiptSet>>. The host returns receipt sets grouped by DHT operation. Each set includes receipts_complete; entry definitions may override the default receipt requirement, so Hearth must not hardcode a validation count. The source documentation specifies that receipt data is available only when the action was authored on the same conductor; it is not necessarily restricted to one agent if that conductor hosts multiple agents/cells for the DNA.

Version-pinned references:
- [HDK 0.6.0 validation_receipt.rs](https://docs.rs/crate/hdk/0.6.0/source/src/validation_receipt.rs)
- [holochain_zome_types 0.6.0 validate.rs](https://docs.rs/crate/holochain_zome_types/0.6.0/source/src/validate.rs)
- [Holochain validation receipts guide](https://developer.holochain.org/build/validation-receipts/)

## Endpoint contract

get_action_publication_evidence(action_hash: ActionHash) -> ExternResult<ActionPublicationEvidence>

The implementation runs inside the Hearth DNA's bridge coordinator zome and queries the local HDK receipt store. Before reading receipt metadata it uses HDK 0.6's local-only get strategy to confirm that the action record is locally available and its author matches the calling agent. Missing and non-owned actions return the same NotAvailableToCaller result, with no receipt metadata, so the endpoint does not become a cross-agent existence/receipt oracle. The HDK host itself can read receipts for other agents on the same conductor; this coordinator wrapper deliberately narrows that capability. The output is a privacy-minimized summary:

- action hash echoed in the response so the caller can bind a report to its query;
- checked_at_micros, sourced from HDK sys_time() after receipt lookup;
- explicit availability = NotAvailableToCaller | Observed | NoOperationData;
- operation count, total receipt-record count, per-operation receipt-record counts, and the number of operation sets whose configured receipt threshold is complete;
- per-operation valid, rejected, and abandoned validation-status counts;
- all_receipt_thresholds_complete, which is explicitly false when no operation sets are returned.

Raw DHT operation hashes and validator identities are intentionally omitted. Receipt-record counts describe the returned HDK receipt records; they must not be presented as a count of unique peers unless the client explicitly counts the returned validators identities under a separately reviewed privacy contract.

## Correct interpretation

- NoOperationData means the receipt query returned no operation sets. It does not mean success, failure, or complete publication.
- Observed means one or more receipt sets were available from this authoring conductor. It says nothing about other conductors.
- receipts_complete = true means Holochain reports the configured receipt threshold for that particular operation. It does not imply all peers in the network possess the operation or that validators remain online.
- rejected_receipt_record_count > 0 flags rejection evidence about that particular DHT operation. It must not retroactively turn the source-chain record into “never committed.”
- Abandoned remains distinct from Rejected; neither state should be hidden by aggregate progress percentages.
- A local signal, successful zome response, or current-state read is not a validation receipt.
- The report is a snapshot, not a continuous health feed. Consumers should render the checked time and mark stale data accordingly.

## Defensive aggregation invariants

The coordinator's pure summarizer is intended to enforce:

1. Total counts equal the sum of per-operation counts, with saturating aggregation rather than integer wraparound.
2. Rejected operation count counts distinct operation sets with one or more rejected receipt records, not the number of rejected receipt records.
3. Valid, rejected, and abandoned receipt-record counts remain separate.
4. Operation threshold completion is copied from the HDK receipts_complete flag, never inferred from a hardcoded count.
5. all_receipt_thresholds_complete is false for an empty result, avoiding vacuous truth.
6. Operation hashes and validator keys do not cross this endpoint.
7. The report is scoped to the current authoring agent, not merely the conductor: a local-only record check must prove the queried action was authored by the caller before receipt data is returned. Missing and non-owned actions share NotAvailableToCaller. When a caller asks about an action authored on another conductor, that conductor-local receipt dataset is not substituted with a network query.

## Test plan and benchmarks

### Pure aggregation tests

- empty operation sets -> NoOperationData, zero counts, and not-complete;
- partial valid/rejected/abandoned receipts -> exact per-operation and aggregate counts;
- multiple operation sets whose thresholds are complete -> complete count based only on receipts_complete;
- serialized summaries contain neither op_hash nor raw validator identities.

### Sweettest qualification

The new ignored conductor integration tests cover:
- querying the action hash returned by a real create_hearth call on the authoring conductor;
- querying an action hash authored on a separate conductor and requiring NotAvailableToCaller rather than a false synchronization claim;
- querying an action authored by another agent on the same conductor and requiring the same non-disclosing NotAvailableToCaller result;
- count consistency, query binding, observation timestamp, and non-vacuous completion.

### Acceptance targets (not results)

- 100% of operation counts and receipt-record counts reconcile with their per-operation summaries.
- Zero empty-result cases labelled complete or published.
- Zero hardcoded required-receipt thresholds.
- Zero global-sync labels based on this API.
- 100% of rejection evidence is scoped to the operation and remains distinct from action/source-chain outcome.
- 100% of reports are bound to their input action hash and carry observation time.
- 100 deterministic replays per fixture produce identical aggregate reports.
- On a controlled multi-agent fixture: measure time-to-first receipt and time-to-each-operation's receipt threshold with sample count and percentiles; do not turn these observations into a delivery SLA.
- At least 90% of usability-test participants distinguish “saved” from “receipt threshold observed” and understand neither guarantees every peer currently has the data.

## Nonclaims

This endpoint does not prove current DHT availability, global consistency, peer liveness, a propagation SLA, or that every peer has observed a record. The receipt sets are observations from the authoring conductor's local receipt store. Passing pure tests proves only the summarizer; only successful exact-head WASM + Sweettest qualification can support implementation claims, and even those qualify only the tested Holochain version/topology.
