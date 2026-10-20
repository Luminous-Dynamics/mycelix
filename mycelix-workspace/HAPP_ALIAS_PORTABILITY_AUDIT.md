# hApp Alias Portability Audit

**Status:** partial portability repair; hApp packaging is not yet qualified.
**Inspection base:** `Luminous-Dynamics/mycelix` main at `e44978ee3ce2c9a53b26a2ffb25952ee98e5840c`.
**Repair PR:** [#5045](https://github.com/Luminous-Dynamics/mycelix/pull/5045).

## Findings

The `mycelix-workspace/happs/` directory contained 20 symlink aliases whose targets were absolute paths rooted at `/srv/luminous-dynamics/mycelix-workspace/`. Such paths depend on one machine's directory layout and do not resolve reliably in a normal clean checkout.

This repair changes the 18 aliases whose canonical sibling source directories are present in the repository to relative links:

- `happs/attribution` → `../mycelix-attribution`
- `happs/civic` → `../mycelix-civic`
- `happs/climate` → `../mycelix-climate`
- `happs/commons` → `../mycelix-commons`
- `happs/core_fl` → `../mycelix-core`
- `happs/craft` → `../mycelix-craft`
- `happs/energy` → `../mycelix-energy`
- `happs/finance` → `../mycelix-finance`
- `happs/governance` → `../mycelix-governance`
- `happs/hearth` → `../mycelix-hearth`
- `happs/identity` → `../mycelix-identity`
- `happs/knowledge` → `../mycelix-knowledge`
- `happs/lawful-identity` → `../mycelix-lawful-identity`
- `happs/marketplace` → `../mycelix-marketplace`
- `happs/music` → `../mycelix-music`
- `happs/personal` → `../mycelix-personal`
- `happs/praxis` → `../mycelix-praxis`
- `happs/manufacturing` → `../mycelix-manufacturing`

## Two unresolved mappings

These paths are still deliberately *not guessed* or redirected to a merely similar cluster:

| Alias | Old target | Finding | Required next step |
|---|---|---|---|
| `happs/health` | `/srv/luminous-dynamics/mycelix-workspace/mycelix-health` | No tracked `mycelix-workspace/mycelix-health/` source exists at the expected path in the inspected tree. The distinct `mycelix-care` hApp is not assumed to be equivalent. | Find the canonical health DNA/source or remove and revise the stale unified-hApp role under an explicit architecture decision. |
| `happs/supplychain` | `/srv/luminous-dynamics/mycelix-workspace/mycelix-supplychain/holochain` | No tracked `mycelix-workspace/mycelix-supplychain/holochain/` source exists at the expected path in the inspected tree. | Locate the intended supply-chain source and its canonical packaging root; then replace the alias and validate manifest paths. |

## Automated guard

`.github/workflows/manufacturing-happ-portability.yml` checks every source-backed symlink under `happs/`: targets must be relative, resolve in the tracked workspace, and not escape the checkout. It names the two unresolved mappings as warnings and fails on any other absolute target, dangling source-backed alias, or alias escaping the workspace.

The check is intentionally narrow: it does not claim that every unified-hApp role is packageable, that the DNA manifests resolve, that Rust/WASM builds pass, or that multi-agent Holochain flows are qualified. The remaining two source mappings need their own source-level fix before the unified hApp can be considered clean-checkout complete.
