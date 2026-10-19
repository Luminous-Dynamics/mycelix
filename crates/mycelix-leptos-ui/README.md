# Mycelix Leptos UI

Shared, public Leptos 0.8 components consumed by `mycelix-leptos-core` and the public Mycelix application crates. This is a real component package, not a resolver-only placeholder.

## Components

- `app_shell`: `AppShell`, `AppNav`, `MobileBottomNav`, `NavLink`, `NavTab`
- `badge`: `Badge`, `BadgeVariant`, `StatusDot`
- `data_table`: `Column`, `DataTable`, `Pagination`
- `empty_state`: `EmptyState`
- `loading`: `LoadingSkeleton`, `Spinner`
- `progress_bar`: `ProgressBar`
- `sovereign_radar`: `SovereignRadar`, `SovereignRadarSize`
- `stat_card`: `StatCard`
- `tabs`: `Tabs`, `TabPanel`
- `toasts`: `Toast`, `ToastKind`, `ToastState`, `ToastContainer`, and context helpers

The components use native HTML controls, semantic table/tab/navigation roles, visible focus states, and touch-friendly primary controls. The shared tabs component supports roving keyboard focus with ArrowLeft/ArrowRight wrapping and Home/End navigation; tab IDs must be unique and match their TabPanel IDs. Reduced-motion preferences are respected for loading effects. The sovereign radar accepts eight finite caller-supplied scores; if the consumer supplies no scores, it reports that profile data is unavailable rather than drawing invented profile values.

## Styling

Applications import the shared base stylesheet explicitly (Trunk does not collect CSS through Rust dependencies):

```css
@import "../../../crates/mycelix-leptos-core/style/base.css";
```

Adjust the relative path to the importing application. The component classes and shared stylesheet live in the public Mycelix repository; no private monorepo source was used to recreate this missing package.

## Validation

From the repository root:

```sh
cargo fmt --manifest-path crates/mycelix-leptos-ui/Cargo.toml -- --check
cargo test --manifest-path crates/mycelix-leptos-ui/Cargo.toml
cargo metadata --manifest-path crates/mycelix-leptos-core/Cargo.toml --format-version 1
cargo check --manifest-path crates/mycelix-leptos-core/Cargo.toml --target wasm32-unknown-unknown
```

A passing component test suite is not itself proof that every application consumer builds. The focused workflow includes a WASM check against the core crate and will keep the original build gate intact.
