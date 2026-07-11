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
      url = "github:holochain/holonix/d21b3543";
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

        # Rust toolchain - reads mycelix-workspace/rust-toolchain.toml (single source of
        # truth), not stable.latest, so devShell builds can't silently drift from the pin
        # and fragment sccache's cache (compiler binary is part of its cache key).
        rustToolchainToml = builtins.fromTOML (builtins.readFile ../rust-toolchain.toml);
        rustChannel = rustToolchainToml.toolchain.channel;
        rustWithWasm = pkgs.rust-bin.stable.${rustChannel}.default.override {
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
            stdenv.cc.cc.lib

            # WASM linker & Optimization
            lld
            llvmPackages.bintools
            binaryen
            trunk

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
            echo "\u{1F331} Mycelix Praxis Hardened Environment"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo ""
            echo "  \u{1F980}  Rust $(rustc --version | cut -d' ' -f2)"
            echo "  \u{1F310}  WASM & PWA tools: ready"
            echo "  \u{1F512}  Linker (LD_LIBRARY_PATH): hardened"
            echo ""
            echo "  Final Quickening Commands:"
            echo "    trunk build --release                      # Build PWA/WASM"
            echo "    cargo test --all                           # E2E Validation"
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # Set environment for dynamic linking (survives non-NixOS)
            export LD_LIBRARY_PATH="${pkgs.stdenv.cc.cc.lib}/lib:$LD_LIBRARY_PATH"
            export LIBCLANG_PATH="${pkgs.libclang.lib}/lib"
          '';

          # Rust environment variables
          RUST_BACKTRACE = "1";
          LIBCLANG_PATH = "${pkgs.libclang.lib}/lib";
          LD_LIBRARY_PATH = "${pkgs.stdenv.cc.cc.lib}/lib";
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
