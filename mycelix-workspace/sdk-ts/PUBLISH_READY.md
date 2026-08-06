# @mycelix/sdk - Publish Readiness

**Version**: 0.6.0
**Status**: NOT ready for NPM publish (corrected 2026-07-27 — this doc previously claimed
"Ready for NPM publish" against version 0.5.0's test count; that was stale even before the
issue below, and is superseded)
**package.json**: `private: true` — `npm publish` is blocked until this is removed

## Known blocker

`npm run typecheck` (`tsc --noEmit`) currently fails with 10 real errors (duplicate
identifier redeclarations in `src/integrations/index.ts`, type mismatches in
`src/integrations/music/index.ts`) — see
[#28](https://github.com/Luminous-Dynamics/luminous-dynamics/issues/28). The vitest suite
mostly passes, but a package that fails `tsc --noEmit` should not be published.

This package has never actually been published to npm (`npm view @mycelix/sdk` returns 404
as of this writing) and currently has no known internal consumers in this monorepo — every
`@mycelix/sdk`-named `package.json` found elsewhere in the tree is either a legacy-archive
copy or a *different* local package that happens to reuse the same npm name, not this one.
The SDK exists for external developer adoption rather than internal dogfooding (this
monorepo's own frontends use Leptos), so publish it once it's genuinely ready, not on a
fixed schedule.

## Publish Checklist (do not treat as complete until re-verified)

- [ ] `npm run typecheck` passes clean (currently 10 errors, see above)
- [ ] All tests passing
- [ ] TypeScript compilation working
- [x] package.json configured
- [x] Exports properly defined
- [x] publishConfig set for public access
- [ ] Remove `private: true` from package.json
- [ ] Remove the pre-release/alpha notice from README.md

## To Publish (once the checklist above is actually complete)

```bash
# Login to NPM (one-time setup)
npm login

# Publish
cd /srv/luminous-dynamics/mycelix-workspace/sdk-ts
npm publish --access public
```

## Package Contents

| Export | Description |
|--------|-------------|
| `@mycelix/sdk` | Main entry point |
| `@mycelix/sdk/matl` | Multi-Agent Trust Layer |
| `@mycelix/sdk/epistemic` | Epistemic validation |
| `@mycelix/sdk/bridge` | Inter-hApp communication |
| `@mycelix/sdk/client` | Holochain client wrapper |
| `@mycelix/sdk/fl` | Federated learning |
| `@mycelix/sdk/errors` | Error types |
| `@mycelix/sdk/security` | Security utilities |
| `@mycelix/sdk/config` | Configuration |
| `@mycelix/sdk/utils` | Utility functions |
