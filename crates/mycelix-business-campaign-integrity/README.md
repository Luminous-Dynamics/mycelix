# mycelix-business-campaign-integrity

Campaign-wide integrity evidence for read-only Mycelix Business imports.

Per-file diagnostics are not enough to prove a multi-file extraction is clean. Two individually valid exports can contain the same provider event, causing one economic event to be counted twice in the campaign denominator and later qualification evidence.

This crate replays the exact registered source-file set and proves that every **accepted** source event is unique across the entire campaign.

## Guarantees

- exact adapter/mapping/source-schema connector identity;
- exact diagnostic-manifest reconstruction for every source file;
- same-file duplicates retain diagnostics' first-accepted / later-rejected semantics;
- an accepted source event repeated in a different file fails closed;
- the globally unique accepted-event count must equal the campaign's accepted-row total;
- observed campaign coverage must equal the aggregate registered manifest coverage;
- exact file identities, event count, event-set digest, coverage, connector, and campaign digest are bound into the evidence digest.

Provider event IDs are used only transiently for uniqueness checking. They are not persisted in `CampaignIntegrityEvidence`; only a deterministic set digest and count are retained.

## Non-claims

This does not prove the upstream POS exported every real event. `upstream-export-completeness-unverified` remains a separate limitation. It also does not treat overlapping time ranges as inherently invalid: two exports may overlap in time while containing distinct events.

The crate is read-only and grants no authority or business write capability.
