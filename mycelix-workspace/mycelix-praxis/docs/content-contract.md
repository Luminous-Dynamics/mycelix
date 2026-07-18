# Praxis curriculum and content contract

Praxis treats curriculum structure and lesson assets as separate inputs joined
by a stable curriculum node ID. The checked-in
`apps/leptos/static/content-manifest.json` is the canonical mapping from an ID
to one or more generated lesson files.

## Invariants

- Every active curriculum document must contain valid JSON.
- Active node IDs must be unique across all embedded documents.
- Both endpoints of every active edge must exist.
- A generated lesson must contain a non-empty `node_id` and a `lesson` object.
- Multiple lesson assets may intentionally target one broad curriculum node.
- The manifest is deterministic and must match the generated lesson tree.
- The application never guesses lesson paths from a node's spelling, grade, or
  subject. Missing content is reported as unavailable.

Run the validator after editing curriculum or generated lessons:

```bash
python3 scripts/validate_curriculum.py --write
python3 scripts/validate_curriculum.py
```

The first command regenerates the manifest. The second is the read-only check
used by CI and Trunk's pre-build hook.

## Coverage is not validity

The validator reports how many active graph nodes have generated lessons, but
does not fail merely because an asset targets a node outside the active graph.
That distinction allows content generation and curriculum curation to proceed
independently without pretending that orphaned content is reachable. Coverage
should rise through deliberate node-ID alignment, not runtime filename
heuristics.

Interactive games use the same node-ID principle through `games::game_type`.
A game is shown only when the active curriculum node resolves to a registered
component key.
