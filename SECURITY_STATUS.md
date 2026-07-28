# Authorization / Author-Binding Remediation Status

An internal audit found a recurring defect class across this codebase: DHT entries and
capabilities in some zomes were not cryptographically bound to the real acting agent
(`agent_info()`), meaning a validator could accept an action attributed to whichever
agent the *client* claimed, not whichever agent actually signed the call. Where present,
this can allow vote forgery, identity/credential takeover, or unauthorized state changes.
Also found repeatedly: a wide-open `RegisterUpdate`/`RegisterDelete` gap (update/delete
validation missing even where create validation was correct).

This document is the canonical, maintained record referenced from mycelix.net's Risks
page. It reflects this repository's own internal engineering tracking, not an
independent third-party audit — treat "fixed" below as "fixed and internally tested,"
not "externally verified."

**Last reviewed:** 2026-07-28, against commit `0c60c66e`.

## Status legend

- **Verified affected, fixed** — defect confirmed present, fix landed, regression test added
- **Verified affected, open** — defect confirmed present, not yet fixed
- **Verified not affected** — checked directly, this cluster's design doesn't have the gap (e.g. no identity fields to bind, or a different authorization model applies)
- **Not yet assessed** — not part of this audit pass yet; absence from this list is not evidence of safety

## Verified affected, fixed

Author-binding closed (with regression coverage) in: `mycelix-attribution`, `mycelix-craft`, `mycelix-core`, `mycelix-personal`, `mycelix-praxis`, `mycelix-property`, `mycelix-water`, `mycelix-identity` (9 zomes), `mycelix-knowledge` (6 zomes), `mycelix-care` (5 zomes), `mycelix-emergency` (7 zomes), `mycelix-health` (7 zomes), `mycelix-housing` (7 zomes), `mycelix-mutualaid` (7 source zomes), `mycelix-media` (5 zomes), `mycelix-manufacturing` (6 zomes), `mycelix-lawful-identity` (3 zomes), `mycelix-position`, `mycelix-space` (including `traffic_control`), `mycelix-commons` (7 non-shadow zomes: circular-marketplace, compost-control, waste-collection, waste-registry, community-calendar, space, support-diagnostics), and civic's `emergency-*`/`media-*`/`mediation` slice (11 zomes, 9 already-clean via a shared gate helper, 2 fixed). Also fixed: an unauthenticated self-mint bug in payments, and a `civitas_dna` privilege-escalation path (`update_causal_reputation` accepted client-supplied agent+score with no caller authentication) — closed via re-derivation from a signature-verified upstream record instead of trusting client input.

## Verified affected, open

- **`threshold_signing_integrity`** — blocked on external `ml-dsa`/`feldman-dkg` dependency drift, not yet fixable without an upstream update.
- **`mycelix-justice`** — Arbitration/Judgment need `must_get`-based juror-authorization (a different, harder pattern than simple author-binding); not yet done.
- **civic cluster** — roughly 28 shadow-zome candidates outside the already-fixed emergency/media slice remain untriaged.
- **`mediation`'s status-transition flow** — a different, functional (not security) bug: its same-author `RegisterUpdate` check is *too strict* for genuine multi-party mediation, not a bypass. Needs a redesign (link/tag-based status instead of entry updates), not a quick bind.

## Verified not affected

- **`mycelix-lawful-identity`** — deliberately has no identity fields on its entries (unlinkability by design); its ownership checks derive from real `agent_info()`-authenticated links, not a self-reported field.

## Not yet assessed for this page's purposes

**`mycelix-governance`** (proposals, voting, threshold-signing, councils, constitution) — the DKG/threshold-signing piece specifically is tracked above as open (blocked on a dependency). Whether the rest of the Governance cluster's zomes (proposals, voting, councils, constitution) have been through this specific author-binding pass has not been independently reconciled for this document. **Do not read "not listed above" as "confirmed safe."** If you're relying on Governance-cluster behavior for anything security-relevant, check this cluster's own zome source and open issues directly rather than trusting this summary.

**Any cluster not named anywhere above** (e.g. `mycelix-finance`'s non-payments zomes, `mycelix-desci`, `mycelix-energy`, `mycelix-climate`) has not been confirmed either way by this document.

## Reporting

See [SECURITY.md](https://github.com/Luminous-Dynamics/.github/blob/main/SECURITY.md) (org-wide policy) for how to report a new finding. Do not open a public issue for an unfixed vulnerability.
