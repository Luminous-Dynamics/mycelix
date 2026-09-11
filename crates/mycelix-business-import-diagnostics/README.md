# mycelix-business-import-diagnostics

Non-mutating extraction diagnostics for read-only Mycelix Business pilots.

The core qualification problem addressed here is denominator integrity. A model or integration must not appear cleaner merely because malformed or unparseable source rows vanished before field evidence was counted.

## Flow

`external export -> bounded dry-run scan -> diagnostic manifest -> extraction campaign -> field evidence`

The scan never admits records to witness state and contains no external write path.

## Manifest guarantees

For each exact source file the manifest binds:

- full connector identity (adapter, mapping, and source-schema digests);
- SHA-256 source-file identity;
- discovered, accepted, and rejected row counts;
- categorized rejection counts without echoing raw row contents;
- observed time range and maximum observed ingest delay;
- supported input classes;
- adapter batch limit and recommended safe chunk count;
- a deterministic manifest digest.

A campaign combines multiple unique file manifests only when their connector identity and supported-input set agree.

## Hospitality field binding

For the read-only hospitality demand pilot:

- `expected_records` must equal the extraction campaign's discovered-row count;
- rejected parser rows form a hard lower bound on `missing_records`;
- reported maximum ingest delay must equal the campaign observation;
- source-schema identity must match preregistration;
- future source timestamps fail closed for this pilot path;
- the evidence must explicitly retain `limitation:upstream-export-completeness-unverified:v1`.

The last rule is intentional. A file scan can detect rows that failed normalization, but it cannot prove that the upstream POS included every real transaction in the export. A future independent completeness witness can discharge that limitation; this crate does not pretend the information exists today.

## Resource bound

Dry-run scans read at most 64 MiB per source file. This prevents a pathological export from turning qualification into an unbounded-memory parser workload. Larger sources should be split into independently digested files or handled by a future streaming-qualified adapter.

Passing this layer still produces only read-only field qualification evidence. It grants no authority and makes no causal financial-benefit claim.
