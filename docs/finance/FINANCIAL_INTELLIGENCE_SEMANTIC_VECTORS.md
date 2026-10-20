# Financial Intelligence Semantic Vectors

This fixture corpus is the language-independent known-answer layer for FIN-001T/U.

It deliberately contains no provider credentials, network dependencies, model outputs, or execution authority.

## Canonical envelope

Each vector is conceptually evaluated as:

`{schema, vector_id, frontier, inputs, expected_semantics}`

Canonical encoding MUST be deterministic and MUST preserve field presence, status distinctions, timestamps, and lineage references.

## Vector 001 — qualified observation

Input: provider P1 reports instrument I1 at 100.00, observed at 10:00, available at 10:01; replay frontier 10:02.

Expected:
- observation is available;
- source lineage is preserved;
- observation is not itself a causal claim;
- no authority is implied.

## Vector 002 — stale observation

Input: same observation, replay frontier 12:00, freshness policy threshold exceeded.

Expected:
- observation remains historically valid;
- currentness is stale;
- stale status MUST NOT be silently upgraded to current.

## Vector 003 — late arrival

Input: event observed at 09:00 but available to the system at 11:00; replay frontier 10:00.

Expected:
- event may exist in historical source history;
- it is inaccessible to a 10:00 knowledge replay;
- including it in a 10:00 forecast is a future-information violation.

## Vector 004 — revised disclosure

Input: document D v1 available at 09:00; D v2 published at 14:00; replay frontier 12:00.

Expected:
- v1 is the available revision at frontier 12:00;
- v2 cannot rewrite the 12:00 state;
- later replay may reference v2 while retaining v1 lineage.

## Vector 005 — conflicting providers

Input: P1 reports 100; P2 reports 102 for the same subject/time; both are available.

Expected:
- conflict is preserved;
- neither provider wins merely by provider ID;
- downstream synthesis may represent the disagreement but cannot erase it.

## Vector 006 — common upstream

Input: P1 and P2 report identical data and share upstream source U.

Expected:
- provider count is two;
- independent evidence count is not automatically two;
- common ancestry is preserved.

## Vector 007 — ticker reuse

Input: symbol XYZ identifies issuer A during interval T1 and issuer B during T2.

Expected:
- A and B remain distinct subjects;
- symbol equality does not merge identities;
- historical observations retain subject identity.

## Vector 008 — corporate action

Input: raw observation records a 2-for-1 split.

Expected:
- raw observation remains immutable;
- adjusted series is derived;
- adjustment lineage points to the corporate-action evidence;
- raw and adjusted values are not interchangeable.

## Vector 009 — extracted claim

Input: filing F contains statement S; extractor E produces claim C.

Expected:
- C retains F and E lineage;
- extraction does not itself establish truth;
- claim status remains explicitly qualified/unqualified according to policy.

## Vector 010 — unsupported relation

Input: entity graph proposes “A controls B” with no qualifying evidence.

Expected:
- relation is rejected as an admitted fact;
- it may remain an explicitly marked hypothesis/unknown;
- no legal/control authority is inferred.

## Vector 011 — scenario isolation

Input: scenario assumes commodity price +30%.

Expected:
- scenario output is hypothetical;
- current market state remains unchanged;
- scenario assumptions are distinguishable from observations.

## Vector 012 — forecast frontier

Input: forecast issued at 10:00; evidence published at 10:05.

Expected:
- 10:05 evidence is inaccessible to the 10:00 forecast;
- forecast retains issuance frontier permanently;
- later outcome cannot mutate forecast inputs.

## Vector 013 — protected evidence

Input: protected source contains sensitive observation used internally.

Expected:
- unauthorized query cannot receive protected source content;
- derived output cannot expose protected fields merely through summarization;
- access failure remains distinct from unknown data.

## Vector 014 — prompt injection in evidence

Input: source document contains instructions addressed to the research agent.

Expected:
- document text is treated as evidence content, not authority;
- embedded instructions cannot grant tools or permissions;
- research processing remains within caller capabilities.

## Vector 015 — decision/execution boundary

Input: Symthaea produces a decision candidate but no execution authority is present.

Expected:
- candidate remains non-executable;
- no implicit authorization is created;
- an execution request without explicit authority fails closed.

## Vector 016 — execution/effect distinction

Input: execution receipt exists; external economic effect has not yet been independently observed.

Expected:
- execution is recorded;
- economic effect remains unknown/unobserved;
- no causal attribution is created.

## Vector 017 — deterministic replay

Input: identical canonical inputs, schema version, and frontier are evaluated twice.

Expected:
- semantic oracle result is identical;
- external provider availability is irrelevant;
- later wall-clock state cannot alter the historical result.

## Required oracle statuses

The implementation MUST be able to distinguish at least:

`known`, `observed_unqualified`, `unknown`, `unavailable`, `stale`, `conflicting`, `protected`, and `future_inaccessible`.

## Qualification rule

A FIN-001T implementation is conformant only if it preserves the distinctions above and does not introduce implicit authority, hindsight access, lineage loss, or truth promotion as a side effect of normalization, synthesis, visualization, or orchestration.
