# Finance source of truth and standalone projection

## Authoritative source

The canonical Finance source is:

`mycelix-workspace/mycelix-finance/`

The public standalone Finance tree:

`mycelix-finance/`

is a projection of that canonical source produced by
`mycelix-workspace/scripts/sync-to-standalone.sh`.

The sync script exports committed monorepo `HEAD` content and maps the
workspace Finance directory into the standalone Finance directory. Therefore
security fixes that are intended to persist must land in the canonical
workspace source before publication.

## Allowed projection differences

The two trees may contain different standalone-only material. Their
`flake.nix` files are intentionally different because the relative path to
the shared Nix modules differs between the monorepo and the standalone
repository.

For every other path that exists in both Finance trees, the tracked content
must be identical.

## Qualification rule

A Finance security change in the public projection without the corresponding
canonical workspace change is not a durable source-of-truth change. It must
either be rejected by the parity gate or be explicitly treated as
projection-only evidence.

The parity guard is implemented in:

`mycelix-workspace/scripts/check-finance-source-parity.sh`

and is intended to run against the exact PR-head SHA during qualification.
