# Authorization / Author-Binding Remediation Status

An internal audit found a recurring defect class across this codebase: some
coordinator functions accepted an agent identity supplied by the *client*
(e.g. as a plain field in the call payload) instead of deriving the caller
from `agent_info()`, and some integrity-zome validators checked a
self-reported "owner" field on an entry instead of binding ownership to the
actual signing author recorded on the DHT operation. Where present, this can
allow vote forgery, identity/credential takeover, or unauthorized state
changes. Also found repeatedly: a wide-open `RegisterUpdate`/`RegisterDelete`
gap (update/delete validation missing even where create validation was
correct).

**Terminology, corrected 2026-07-28:** `agent_info()` is available to
coordinator zomes, not inside deterministic integrity validation. The two
correct fix patterns are distinct:
- **Coordinator functions** must derive the local caller from `agent_info()`
  rather than trusting a client-supplied identity field, before writing any
  entry.
- **Integrity-zome validation** must bind self-reported ownership fields and
  update/delete authorization to the *author of the signed action* itself
  (via `must_get_action`/`must_get_valid_record` retrieving the original
  record deterministically), not to a value the entry happens to contain.

This document is the canonical, maintained record referenced from
mycelix.net's Risks page. It reflects this repository's own internal
engineering tracking, not an independent third-party audit — treat "fixed"
below as "fixed and internally tested," not "externally verified."

**Last reviewed:** 2026-07-28, against
[commit `bd97865e`](https://github.com/Luminous-Dynamics/mycelix/commit/bd97865e8cffb730a284b8a386c613df9ed58db9)
(2026-07-28) on `main`. An earlier version of this document cited `0c60c66e`
as "reviewed 2026-07-28" — that commit is actually dated 2026-07-18; this was
a real dating error in the previous revision, corrected here, not a claim
that anything changed between the two commits.

## Status legend

- **Fixed internally** — defect confirmed present, fix landed, regression test added, not independently/externally verified
- **Open** — defect confirmed present, not yet fixed
- **Not affected** — checked directly, this component's design doesn't have the gap (e.g. no self-reported identity field to bind, or a different authorization model applies)
- **Not yet assessed** — not part of this audit pass yet; absence from this table is not evidence of safety

## Remediated and not-affected components

Per-fix commit hashes, exact test names, and reviewed-revision pins are **not
tracked at this granularity in current internal records** for most rows below
— this table reports what's known now rather than waiting to backfill exact
commit SHAs before publishing anything. Where a specific commit is on record,
it's given; where not, the cell says so honestly rather than inventing one.
Check each cluster's own PR/commit history on GitHub for the authoritative
detail behind any row.

| Component | Status | Defect pattern | Enforcement point | Fix commit | Regression test | Reviewed at |
|---|---|---|---|---|---|---|
| `mycelix-attribution` | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-craft` | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-core` | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-personal` | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-praxis` | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-property` | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-water` | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-identity` (9 zomes) | Fixed internally | Mixed: client-supplied identity + unbound update/delete | Coordinator binding + integrity author-match | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-knowledge` (6 zomes) | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-care` (5 zomes) | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-emergency` (7 zomes) | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-health` (7 zomes) | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-housing` (7 zomes) | Fixed internally | Client-supplied author field + real privilege-escalation (board/lease takeover) | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-mutualaid` (7 source zomes) | Fixed internally | Client-supplied author field (AgentPubKey-based zomes only; String-DID zomes disclosed, not fixed) | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-media` (5 zomes) | Fixed internally | 2 of 5 coordinators had **zero** `agent_info()` calls anywhere | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-manufacturing` (6 zomes) | Fixed internally | Unbound update, zero identity fields (shared shop-floor system) | `must_get`-based content restriction | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-position` | Fixed internally | Client-supplied author field | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-space` (incl. `traffic_control`) | Fixed internally | Client-supplied author field + self-grant privilege escalation | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `mycelix-commons` (7 non-shadow zomes: circular-marketplace, compost-control, waste-collection, waste-registry, community-calendar, space, support-diagnostics) | Fixed internally | Client-supplied author field; `community-calendar` had zero authorization on its entire entry set | Coordinator `agent_info()` binding | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| civic `emergency-*`/`media-*`/`mediation` slice (11 zomes) | Fixed internally (2 of 11) / already clean (9 of 11) | Self-reported `requester_did` vs. self-reported `author_did`, both attacker-controlled | Shared `check_author_match`/`gate_civic` helper, real `agent_info()` | `e104512f02`, `dd4aed2215` | present, name not tracked here | 2026-07-27 |
| `mycelix-care-credentials` | Fixed internally | Unbound `holder` field; self-declared `verified` bypassed higher-trust gate | `agent_info()` binding + create-time restriction | `48a460dcf` (first fix, lost/re-landed) | 94/94 pass | 2026-07-26 |
| payments (self-mint) | Fixed internally | Unauthenticated self-mint | Authenticated mint path | not tracked at this granularity | present, name not tracked here | 2026-07 pass |
| `civitas_dna` | Fixed internally | `update_causal_reputation` accepted client-supplied agent+score with no caller authentication; root cause traced one level upstream to `record_contribution` | Re-derivation from a signature-verified upstream record via `must_get_valid_record`, not trusted client input | `33fb1630b6`, `f82d949ec5` | 9/9 new + 3/3 pre-existing pass | 2026-07-28 |
| `mycelix-lawful-identity` | Not affected (author-binding); separately fixed (RegisterUpdate/RegisterDelete) | No self-reported identity field exists to bind at all — deliberate unlinkability design, not an oversight. Ownership derives from real `agent_info()`-authenticated links. The unrelated wide-open RegisterUpdate/RegisterDelete gap (present in nearly every cluster audited) was still closed here as a matter of course. | Authenticated links (not a field) | not tracked at this granularity | present, name not tracked here | 2026-07 pass |

## Open

| Component | Status | Notes |
|---|---|---|
| `threshold_signing_integrity` | Open | Blocked on external `ml-dsa`/`feldman-dkg` dependency drift — not fixable without an upstream update |
| `mycelix-justice` | Open | Arbitration/Judgment need `must_get`-based juror-authorization — a different, harder pattern than simple author-binding; not started |
| civic cluster (remaining) | Open, count not yet reconciled | Shadow-zome candidates outside the already-fixed emergency/media/mediation slice — see [open issues](https://github.com/Luminous-Dynamics/mycelix/issues) for the current unassessed-path list rather than trusting a headline count here |
| `mediation` status-transition flow | Open (functional bug, not a security bypass) | Its same-author `RegisterUpdate` check is *too strict* for genuine multi-party mediation — needs a redesign (link/tag-based status instead of entry updates), not a quick bind |

## Not yet assessed for this document's purposes

**`mycelix-governance`** (proposals, voting, threshold-signing, councils,
constitution) — the DKG/threshold-signing piece specifically is listed above
as Open (blocked on a dependency). Whether the rest of the Governance
cluster's zomes (proposals, voting, councils, constitution) have been through
this specific author-binding pass has not been independently reconciled for
this document. **Do not read "not listed above" as "confirmed safe."** If
you're relying on Governance-cluster behavior for anything security-relevant,
check this cluster's own zome source and open issues directly rather than
trusting this summary.

**Any component not named anywhere above** (e.g. `mycelix-finance`'s
non-payments zomes, `mycelix-desci`, `mycelix-energy`, `mycelix-climate`) has
not been confirmed either way by this document.

## Reporting

See [SECURITY.md](https://github.com/Luminous-Dynamics/.github/blob/main/SECURITY.md)
(org-wide policy) for how to report a new finding. Do not open a public issue
for an unfixed vulnerability.
