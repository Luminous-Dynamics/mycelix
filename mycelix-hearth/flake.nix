# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
# Mycelix Hearth - Family/Household/Kinship Coordination
# HEARTH tier: shared family space between ME (Personal) and WE (Civic)
#
# Usage:
#   nix develop              # Enter dev shell
#   nix develop .#ci         # CI environment (minimal)
{
  description = "Mycelix Hearth - Family/household coordination on Holochain";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";

    holonix = {
      url = "github:holochain/holonix/d21b3543";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, holonix, rust-overlay }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
          config.allowUnfree = true;
        };

        holochainPackages = holonix.packages.${system};

        rustToolchain = pkgs.rust-bin.stable."1.96.0".default.override {
          targets = [ "wasm32-unknown-unknown" ];
          extensions = [ "rust-src" "rust-analyzer" "clippy" ];
        };

        libclangPath = "${pkgs.llvmPackages.libclang.lib}/lib";
        clangResourceDir =
          "${pkgs.llvmPackages.clang.cc}/lib/clang/${pkgs.lib.versions.major pkgs.llvmPackages.clang.version}/include";
        bindgenArgs = builtins.concatStringsSep " " [
          "-I${pkgs.glibc.dev}/include"
          "-I${clangResourceDir}"
        ];

        commonBuildInputs = with pkgs; [
          rustToolchain
          holochainPackages.holochain
          holochainPackages.hc
          pkg-config
          openssl
          openssl.dev
          cmake
          gnumake
          llvmPackages.libclang
          llvmPackages.clang
          glibc.dev
        ];

        hearthEnv = {
          LIBCLANG_PATH = libclangPath;
          BINDGEN_EXTRA_CLANG_ARGS = bindgenArgs;
          OPENSSL_DIR = "${pkgs.openssl.dev}";
          OPENSSL_LIB_DIR = "${pkgs.openssl.out}/lib";
          OPENSSL_INCLUDE_DIR = "${pkgs.openssl.dev}/include";
          PKG_CONFIG_PATH = "${pkgs.openssl.dev}/lib/pkgconfig";
          RUST_BACKTRACE = "1";
          RUST_LOG = "info";
        };

        shellHook = ''
          echo "Mycelix Hearth — Family/Household/Kinship Coordination"
          echo "Holochain: $(holochain --version 2>/dev/null || echo 'loading...')"
          echo "hc:        $(hc --version 2>/dev/null || echo 'loading...')"
          echo "Rust:      $(rustc --version)"
        '';

      in {
        devShells = {
          default = pkgs.mkShell ({
            name = "mycelix-hearth";
            buildInputs = commonBuildInputs ++ [ pkgs.nodejs_20 ];
            inherit (hearthEnv)
              LIBCLANG_PATH BINDGEN_EXTRA_CLANG_ARGS
              OPENSSL_DIR OPENSSL_LIB_DIR OPENSSL_INCLUDE_DIR
              PKG_CONFIG_PATH RUST_BACKTRACE RUST_LOG;
            inherit shellHook;
          });

          ci = pkgs.mkShell ({
            name = "mycelix-hearth-ci";
            buildInputs = commonBuildInputs;
            inherit (hearthEnv)
              LIBCLANG_PATH BINDGEN_EXTRA_CLANG_ARGS
              OPENSSL_DIR OPENSSL_LIB_DIR OPENSSL_INCLUDE_DIR
              PKG_CONFIG_PATH RUST_BACKTRACE RUST_LOG;
          });
        };

        packages = {
          zomes = pkgs.stdenv.mkDerivation {
            name = "mycelix-hearth-zomes";
            src = ./.;
            nativeBuildInputs = [ rustToolchain pkgs.pkg-config ];
            buildInputs = [ pkgs.openssl pkgs.openssl.dev ];
            buildPhase = ''
              export HOME=$TMPDIR
              cargo build --release --target wasm32-unknown-unknown --workspace
            '';
            installPhase = ''
              mkdir -p $out/lib
              find target/wasm32-unknown-unknown/release -name "*.wasm" -exec cp {} $out/lib/ ;
            '';
          };
        };
      }
    );
}
