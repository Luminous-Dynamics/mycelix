# Mycelix Pulse surface status

This inventory defines support boundaries for the working alpha. Directory
presence does not imply product support.

| Surface | Status | Alpha responsibility |
|---|---|---|
| `apps/leptos` | Canonical | Browser UI and live conductor integration |
| `holochain` | Canonical | DNA, integrity rules, coordinator behavior, client fixtures |
| `crates/mail-leptos-types` | Canonical | Browser-safe views and versioned protocol contracts |
| `tests/sweettest_mail_security.rs` | Canonical | Deterministic multi-agent lifecycle evidence |
| `services/pulse-credential-broker` | Canonical | Authenticated, no-store token and signer boundary |
| workspace `crates/mycelix-leptos-*` | Canonical dependency | Shared transport and UI primitives |
| `desktop` | Experimental | Optional shell; no alpha support claim |
| `crates/pulse-smtp-gateway` | Experimental | Independently tested; excluded from alpha claims |
| `ui/frontend`, `ui/backend` | Frozen | React/Node reference implementation; no new features |
| `happ` | Frozen | Legacy DNA/backend/CLI reference implementation |
| `backend/api`, `backend/ml` | Frozen | Parallel API prototypes outside the canonical data path |
| `mobile`, `extension`, `frontend`, SDK prototypes | Frozen | Not built or released for the alpha |
| `_deprecated`, `_archive_react_ui` | Deprecated | Historical reference only |
| duplicate `deploy` and `k8s` manifests | Frozen | Must not be represented as a supported deployment |

Frozen surfaces remain available until the canonical headless lifecycle passes.
Before archival, inventory unique behavior, port alpha-required behavior, and
create a reproducible repository tag. They are excluded from canonical CI now.
