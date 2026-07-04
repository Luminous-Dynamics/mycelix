# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
{
  description = "Mycelix EduNet - Privacy-Preserving Decentralized Education Platform";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    holonix = {
      url = "github:holochain/holonix?ref=main-0.6";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, rust-overlay, holonix }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };

        # Rust with WASM target included
        rustWithWasm = pkgs.rust-bin.stable.latest.default.override {
          targets = [ "wasm32-unknown-unknown" ];
        };

        # Holochain 0.6 packages
        holonixPkgs = holonix.packages.${system};
      in
      {
        devShells.default = pkgs.mkShell {
          name = "mycelix-edunet-dev";

          buildInputs = with pkgs; [
            # Rust with WASM support
            rustWithWasm

            # Build tools
            pkg-config
            openssl
            libclang.lib

            # WASM linker (required for wasm32-unknown-unknown target)
            lld
            llvmPackages.bintools

            # Holochain 0.6 tools
            holonixPkgs.holochain
            holonixPkgs.hc
            holonixPkgs.lair-keystore
            holonixPkgs.hc-scaffold

            # Development utilities
            cargo-watch
            cargo-edit
          ];

          shellHook = ''
            echo ""
            echo "🎓 Mycelix EduNet Development Environment"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo ""
            echo "  🦀  Rust $(rustc --version | cut -d' ' -f2)"
            echo "  🌐  WASM target: available"
            echo "  📦  Workspace members: 10"
            echo ""
            echo "  Quick commands:"
            echo "    cargo build --release                      # Build all"
            echo "    cargo build --target wasm32-unknown-unknown --release  # Build WASM"
            echo "    cargo test --all                           # Run tests"
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  Phase 7: E2E Testing ready"
            echo ""

            # Set LIBCLANG_PATH for bindgen
            export LIBCLANG_PATH="${pkgs.libclang.lib}/lib"
          '';

          # Rust environment variables
          RUST_BACKTRACE = "1";
          LIBCLANG_PATH = "${pkgs.libclang.lib}/lib";
        };

        # Native Linux deps for `apps/tauri/src-tauri` (Tauri v2 desktop
        # client). Deliberately separate from devShells.default: that shell
        # pulls in the full holonix Holochain toolchain, which is unrelated
        # to (and much heavier than) what's needed to just build the
        # desktop wrapper. `cargo check`/`cargo build` here needs
        # webkit2gtk + dbus (Tauri's tray/menu backend links libdbus
        # directly, not just via zbus) — without these, the build fails
        # with "Package dbus-1 was not found in the pkg-config search path".
        devShells.tauri-desktop = pkgs.mkShell {
          name = "mycelix-praxis-tauri-dev";

          buildInputs = with pkgs; [
            rustWithWasm
            pkg-config
            gtk3
            webkitgtk_4_1
            libsoup_3
            librsvg
            dbus
          ];
        };
      });
}
