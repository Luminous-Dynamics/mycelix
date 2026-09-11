# mycelix-business-campaign-replay

Canonical bounded read-only replay for Business qualification.

New qualification layers should consume this crate rather than invent another CSV-row interpretation. It reconstructs the exact registered extraction campaign and emits only deterministic normalized observations from accepted rows.

## Replay contract

The replay:

- requires the exact registered file set and original ingestion timestamps;
- reruns diagnostics and requires every regenerated manifest to equal the registered manifest;
- preserves first-accepted / later-rejected duplicate semantics within one file;
- rejects accepted source-event reuse across files;
- requires accepted-event conservation against the campaign manifest;
- returns normalized observations together with their declared input identity;
- binds the exact connector, file set, accepted-event count, normalized-observation count, and observation-set digest into evidence.

Malformed source rows remain rejected; replay does not make them disappear from the extraction denominator.

## Resource bounds

Replay is bounded to 4,096 source files and 2,000,000 normalized observations per invocation. Each source file remains subject to the delimited adapter/diagnostic 64 MiB input bound.

These are implementation limits, not economic semantics. Larger campaigns should be partitioned into independently qualified replay epochs or handled by a future streaming replay interface.

## Privacy and authority

The replay result can contain normalized witness observations for immediate downstream computation, but the compact evidence record contains only counts/digests and connector/file identities. This crate carries no credentials, performs no provider mutation, and grants no authority.
