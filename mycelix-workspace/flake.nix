# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
# Mycelix Ecosystem - Unified Development Environment
# Orchestrates all Mycelix hApps with shared Holochain infrastructure
#
# Usage:
#   nix develop              # Full dev environment (all hApps)
#   nix develop .#holochain  # Holochain-only (zome development)
#   nix develop .#ml         # Python ML/FL environment
#   nix develop .#ci         # CI environment (minimal)
#
# Individual hApps (use their own flakes for focused work):
#   cd ../mycelix-finance && nix develop
#   cd ../mycelix-identity && nix develop
#   cd ../mycelix-governance && nix develop
#   cd ../mycelix-knowledge && nix develop
{
  description = "Mycelix Ecosystem - Unified development environment for all hApps";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";

    # Holochain from holonix — version pinned in nix/modules/holochain-versions.nix
    # To upgrade: update holochain-versions.nix, then `nix flake lock --update-input holonix`
    holonix = {
      url = "github:holochain/holonix/d21b3543"; # Must match holonixCommit in holochain-versions.nix
      inputs.nixpkgs.follows = "nixpkgs";
    };

    # Rust toolchain
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, holonix, rust-overlay }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [
          (import rust-overlay)
          # Fix flaky test failures in nixpkgs python packages
          (final: prev: {
            python311 = prev.python311.override {
              packageOverrides = pyself: pysuper: {
                websockets = pysuper.websockets.overridePythonAttrs (old: {
                  doCheck = false;
                });
              };
            };
          })
        ];
        pkgs = import nixpkgs {
          inherit system overlays;
          config.allowUnfree = true;
        };

        # Holochain packages from holonix
        holochainPackages = holonix.packages.${system};

        # Import shared Holochain base configuration
        holochainBase = import ../nix/modules/holochain-base.nix {
          inherit pkgs system;
          holochainPackages = holochainPackages;
        };

        # Python environment for 0TML (Federated Learning)
        pythonEnv = pkgs.python311.withPackages (ps: with ps; [
          numpy
          scipy
          pandas
          torch
          torchvision
          scikit-learn
          matplotlib
          pytest
          pytest-asyncio
          black
          mypy
          ruff
          # Async
          aiohttp
          websockets
        ]);

        # Node.js packages
        nodeEnv = with pkgs; [
          nodejs_24
          nodePackages.pnpm
          nodePackages.typescript
          nodePackages.typescript-language-server
        ];

        # Keep this focused SDK shell self-contained within the flake source.
        # The general Holochain shells intentionally reuse ../nix/modules, which
        # is outside this nested flake root; the SDK shell must not require
        # impure evaluation just to obtain its toolchain.
        sdkRustToolchain = pkgs.rust-bin.stable."1.96.0".default.override {
          targets = [ "wasm32-unknown-unknown" ];
          extensions = [ "rust-src" "rust-analyzer" "clippy" "rustfmt" ];
        };
        sdkClangResourceDir = "${pkgs.llvmPackages.clang.cc}/lib/clang/${pkgs.lib.versions.major pkgs.llvmPackages.clang.version}/include";
        sdkBindgenArgs = builtins.concatStringsSep " " [
          "-I${pkgs.glibc.dev}/include"
          "-I${sdkClangResourceDir}"
        ];

        # Resolve the committed package-lock.json into Nix-store dependency paths.
        # For a package build, use buildNpmPackage + importNpmLock's package hook;
        # the buildNodeModules/linkNodeModulesHook pair is for dev-shell workflows.
        # No second package-manager lockfile or hand-maintained dependency hash is
        # needed: importNpmLock consumes the committed lockfile integrity metadata.
        sdkTsPackage = builtins.fromJSON (builtins.readFile ./sdk-ts/package.json);
        sdkTsDependencies = pkgs.importNpmLock {
          npmRoot = ./sdk-ts;
        };
        sdkTs = pkgs.buildNpmPackage {
          pname = "mycelix-sdk-ts";
          version = sdkTsPackage.version;
          src = ./sdk-ts;
          npmDeps = sdkTsDependencies;
          npmConfigHook = pkgs.importNpmLock.npmConfigHook;
          # Pin the runtime used by the builder and fail closed on missing local deps.
          nodejs = pkgs.nodejs_24;
          npm_config_offline = "true";
          npm_config_audit = "false";
          npm_config_fund = "false";

          # Run the quality gates before the normal npm build hook runs "build".
          preBuild = ''
            npm run typecheck
            npm run lint
            npm test
          '';

          installPhase = ''
            runHook preInstall
            mkdir -p "$out"
            cp -r dist "$out/dist"
            cp package.json README.md LICENSE "$out/"
            runHook postInstall
          '';

          meta = {
            description = "Nix-built Mycelix TypeScript SDK; typecheck, lint, tests and build are required";
            platforms = pkgs.nodejs_24.meta.platforms;
          };
        };

      in {
        devShells = {
          # Full development environment (all tools)
          default = holochainBase.mkHolochainShell {
            name = "mycelix-workspace";
            extraBuildInputs = nodeEnv ++ [ pythonEnv ] ++ (with pkgs; [
              # Documentation
              mdbook
              graphviz

              # Additional dev tools
              httpie
              websocat
              gh
            ]);
            extraShellHook = ''
              echo ""
              echo "Mycelix Ecosystem Workspaces:"
              echo ""
              echo "  Core hApps (use individual flakes for focused work):"
              echo "    ../mycelix-finance/    - CGC, TEND, HEARTH, Treasury"
              echo "    ../mycelix-identity/   - DID, Verifiable Credentials"
              echo "    ../mycelix-governance/ - Proposals, Voting, Constitution"
              echo "    ../mycelix-knowledge/  - Epistemic Claims, Knowledge Graph"
              echo ""
              echo "  This Workspace:"
              echo "    ./sdk/                 - Shared Mycelix SDK"
              echo "    ./happs/               - Application hApps"
              echo "    ./core/0tml/           - Byzantine FL (Zero-TrustML)"
              echo "    ./observatory/         - Unified Dashboard"
              echo ""
              echo "Quick Commands:"
              echo "  just build               - Build all zomes"
              echo "  just test                - Run all tests"
              echo "  just dev                 - Start development servers"
              echo ""

              # Python venv for 0TML
              if [ -d "core/0tml" ] && [ ! -d "core/0tml/.venv" ]; then
                echo "Tip: Run 'python -m venv core/0tml/.venv' for 0TML development"
              fi
            '';
          };

          # CI environment (minimal, fast to build)
          ci = pkgs.mkShell {
            name = "mycelix-ci";
            buildInputs = with pkgs; [
              holochainPackages.holochain
              holochainPackages.hc
              holochainBase.rustToolchain
              nodejs_24
              nodePackages.pnpm
              pythonEnv
              just
              pkg-config
              openssl
              openssl.dev
            ];

            inherit (holochainBase.envVars)
              LIBCLANG_PATH BINDGEN_EXTRA_CLANG_ARGS
              OPENSSL_DIR OPENSSL_LIB_DIR OPENSSL_INCLUDE_DIR;
          };

          # Focused SDK CI environment: pinned Rust and Node, without the full
          # Holochain/Python ML development closure used by .#ci. This output
          # deliberately avoids the parent-directory module import so pure
          # flake evaluation works when this nested flake is used directly.
          sdk-ci = pkgs.mkShell {
            name = "mycelix-sdk-ci";
            buildInputs = [
              sdkRustToolchain
              pkgs.nodejs_24
              pkgs.pkg-config
              pkgs.openssl
              pkgs.openssl.dev
              pkgs.llvmPackages.libclang
              pkgs.llvmPackages.clang
              pkgs.glibc.dev
              pkgs.stdenv.cc
            ];

            LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
            BINDGEN_EXTRA_CLANG_ARGS = sdkBindgenArgs;
            OPENSSL_DIR = "${pkgs.openssl.dev}";
            OPENSSL_LIB_DIR = "${pkgs.openssl.out}/lib";
            OPENSSL_INCLUDE_DIR = "${pkgs.openssl.dev}/include";
            PKG_CONFIG_PATH = "${pkgs.openssl.dev}/lib/pkgconfig";
          };

          # Holochain-only environment (focused zome development)
          holochain = holochainBase.mkHolochainShell {
            name = "mycelix-holochain";
            extraShellHook = ''
              echo "Holochain-focused development environment"
              echo "For full tools, use 'nix develop' (default shell)"
            '';
          };

          # Python ML environment (0TML development)
          ml = pkgs.mkShell {
            name = "mycelix-ml";
            buildInputs = [ pythonEnv pkgs.just ];

            shellHook = ''
              echo "=========================================="
              echo "  Mycelix ML/FL Development Environment"
              echo "=========================================="
              echo ""
              echo "Python: $(python --version)"
              echo ""
              echo "0TML (Zero-TrustML) development:"
              echo "  cd core/0tml"
              echo "  pytest                   - Run FL tests"
              echo "  python -m mycelix_fl     - Run FL node"
              echo ""
            '';
          };

          # Documentation environment
          docs = pkgs.mkShell {
            name = "mycelix-docs";
            buildInputs = with pkgs; [
              nodejs_24
              nodePackages.pnpm
              mdbook
              graphviz
            ];

            shellHook = ''
              echo "Documentation environment ready!"
              echo "  mdbook serve docs/       - Serve Rust docs"
              echo "  cd observatory && pnpm dev - Dev dashboard"
            '';
          };
        };

        # Packages
        packages = {
          # Build, typecheck, lint, and test the TypeScript SDK entirely from
          # the flake-pinned Node toolchain and Nix-materialized lockfile.
          sdk-ts = sdkTs;

          # Build all core zomes from workspace
          all-zomes = pkgs.stdenv.mkDerivation {
            name = "mycelix-all-zomes";
            src = ../.;

            nativeBuildInputs = [
              holochainBase.rustToolchain
              pkgs.pkg-config
            ];

            buildInputs = [ pkgs.openssl ];

            buildPhase = ''
              export HOME=$TMPDIR

              # Build each hApp's zomes
              for dir in mycelix-finance mycelix-identity mycelix-governance mycelix-knowledge; do
                if [ -d "$dir" ] && [ -f "$dir/Cargo.toml" ]; then
                  echo "Building $dir..."
                  (cd "$dir" && cargo build --release --target wasm32-unknown-unknown) || true
                fi
              done
            '';

            installPhase = ''
              mkdir -p $out/lib
              find . -path "*/target/wasm32-unknown-unknown/release/*.wasm" -exec cp {} $out/lib/ \;
            '';
          };
        };

        # Include the same derivation in nix flake check; packages.sdk-ts is
        # also available for direct builds and CI evidence collection.
        checks.sdk-ts = sdkTs;
      }
    );
}
