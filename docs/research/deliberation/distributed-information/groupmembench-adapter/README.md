# GroupMemBench read-only adapter v0.1

**Status:** implemented parser and local identity verifier; external payload not downloaded or executed.

## Purpose

Provide a deterministic, non-mutating intake adapter for the pinned synthetic GroupMemBench release. It verifies local checkout identity and exact Git blob SHA-1/byte-size values against `GROUPMEMBENCH_METADATA_INTAKE_V0_1.json`, then provides candidate-safe projections for messages and questions.

## Upstream identity and rights

- Repository: https://github.com/UCSB-NLP-Chang/GroupMemBench
- Commit: `461f00c3e8a1a4a3a2ed5814b729014502c4702f`
- Code license declared upstream: MIT.
- Conversation and question data declared upstream: CC BY 4.0.
- The manifest records 30 selected file identities; this adapter requires the exact pinned checkout before local verification.
- This package does not download, copy, upload, redistribute or modify upstream payloads.

## Data mapping

The upstream conversation file is a JSON object keyed by channel, each with an ordered array of messages. The adapter preserves the baseline's deterministic ordering by `(channel, timestamp, msg_node)` and exposes only:
- `msg_node`, `content`, `author`, `role`, `timestamp`;
- optional `phase_name`, `topic`, `reply_to`;
- channel identity.

Evaluator-side fields such as `is_noise`, `is_decision_point`, tone/expertise labels and decision metadata are not projected into candidate views.

Question JSONL records are split into:
- candidate view: `id`, `question`, `asking_user_id`;
- evaluator-only answer: held separately in the adapter's `QuestionRecord`.

This prevents answer leakage while preserving audience/implicit-reference identity. No evaluator output is sent to a candidate.

## Read-only local verification

After locally obtaining the exact pinned upstream checkout under its declared license terms:

```bash
cargo test --manifest-path docs/research/deliberation/distributed-information/groupmembench-adapter/Cargo.toml

cargo run --manifest-path docs/research/deliberation/distributed-information/groupmembench-adapter/Cargo.toml -- \
  docs/research/deliberation/distributed-information/GROUPMEMBENCH_METADATA_INTAKE_V0_1.json \
  /path/to/GroupMemBench
```

The CLI checks:
- exact Git HEAD commit;
- all 30 pinned paths;
- Git blob SHA-1 computed as `SHA1("blob " + size + NUL + bytes)`;
- exact file byte lengths;
- path traversal and symlink escape.

It prints an intake summary only. It does not write into the upstream checkout or import payloads into Mycelix.

## Boundaries

This is an adapter/parser and file-identity verification contract. It does not run BM25, dense retrieval, a model, or the 715 questions. A separate benchmark run must pin agent/judge model versions, tokenizer, top-k, prompt, compute/token budgets and output schema. Compare at minimum BM25, dense retrieval, full-context and the upstream reported baselines under matched conditions.

No claim is made that candidate-safe projection alone preserves all GroupMemBench scoring semantics; the adapter must be compared byte-for-byte against the upstream loader/formatting contract before any benchmark result is claimed.

 
## Deterministic BM25 baseline

The adapter now implements the upstream GroupMemBench lexical baseline contract:
- ASCII word tokenizer matching [A-Za-z0-9_]+, lowercase, no stopword removal;
- HTML entity decoding and trimming before indexing;
- content-only indexing (speaker/channel/role/time metadata do not affect scores);
- BM25Okapi defaults k1=1.5, b=0.75, epsilon=0.25;
- stable descending-score ranking with original message index as tie-break;
- read-time passage formatting with author, role, channel, phase, topic, timestamp, reply_to and msg_node.

This is a local CPU implementation of the retrieval method, not an executed result on the upstream corpus. It can be used after local pinned-checkout verification to compare BM25 against a proposed evidence-aware memory method with identical top-k and context budgets. It does not call an LLM.


## Optional exact-corpus BM25 process probe

The read-only verification job also runs `--bm25-probe` after validating the actual pinned checkout. It builds four content-only BM25 indexes, streams all 715 question records, and reports per-domain message count, vocabulary size, question count, lexical-overlap question count and top-10 passage count. It intentionally does not evaluate answer accuracy or read/use the gold answers. This is a retrieval-process baseline, not a benchmark score or model result.
