{
  description = "Mycelix Marketplace - development and promotion tooling";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.05";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        nodejs = pkgs.nodejs_20;
        frontendSrc = ./frontend;
        npmDepsHash = "sha256-7FNbB1/niXZDSAyB6Q+K117tK6obQbYH9csNY4oAtcw=";
        repoApp = name: script: description:
          (flake-utils.lib.mkApp {
            drv = pkgs.writeShellApplication {
              inherit name;
              runtimeInputs = [ pkgs.git pkgs.bash ];
              text = ''
                repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
                exec "$repo_root/scripts/${script}" "$@"
              '';
            };
          }) // { meta.description = description; };
        promotionPackages = with pkgs; [
          rustup
          nodejs
          python3
          jq
          openssl
          binaryen
          lld
          gcc
          pkg-config
          git
          bash
          coreutils
          findutils
          gnused
          gawk
          netcat-openbsd
          procps
        ];
      in {
        packages = {
          frontend = pkgs.buildNpmPackage {
            pname = "mycelix-marketplace-frontend";
            version = "1.0.0";
            src = frontendSrc;
            npmDepsHash = npmDepsHash;
            npmBuildScript = "build";
            installPhase = ''
              runHook preInstall
              mkdir -p $out
              if [ -d .vercel/output ]; then
                cp -r .vercel/output/. "$out/"
              elif [ -d build ]; then
                cp -r build/. "$out/"
              elif [ -d .svelte-kit ]; then
                cp -r .svelte-kit "$out/.svelte-kit"
              else
                echo "warning: no known frontend build artifact produced" >&2
              fi
              runHook postInstall
            '';
          };
          promotion-tools = pkgs.buildEnv {
            name = "mycelix-marketplace-promotion-tools";
            paths = promotionPackages;
          };
          default = self.packages.${system}.frontend;
        };

        checks = {
          frontend-check = pkgs.buildNpmPackage {
            pname = "mycelix-marketplace-frontend-check";
            version = "1.0.0";
            src = frontendSrc;
            npmDepsHash = npmDepsHash;
            npmBuildScript = "check";
            installPhase = ''mkdir -p $out; touch $out/done'';
          };
          frontend-lint = pkgs.buildNpmPackage {
            pname = "mycelix-marketplace-frontend-lint";
            version = "1.0.0";
            src = frontendSrc;
            npmDepsHash = npmDepsHash;
            npmBuildScript = "lint";
            installPhase = ''mkdir -p $out; touch $out/done'';
          };
          frontend-test = pkgs.buildNpmPackage {
            pname = "mycelix-marketplace-frontend-test";
            version = "1.0.0";
            src = frontendSrc;
            npmDepsHash = npmDepsHash;
            npmBuildScript = "test";
            installPhase = ''mkdir -p $out; touch $out/done'';
          };
        };

        apps = {
          check = repoApp "mycelix-check" "validate-leptos-migration.sh" "Run Marketplace validation gates";
          promotion-check = repoApp "marketplace-promotion-check" "check-promotion-environment.sh" "Verify the promotion toolchain and repository state";
          promotion-build = repoApp "marketplace-promotion-build" "build-promotion-artifacts.sh" "Build exact Marketplace promotion artifacts";
          promotion-run = repoApp "marketplace-promotion-run" "run-disposable-promotion.sh" "Run a disposable-conductor promotion profile";
          promotion-verify = repoApp "marketplace-promotion-verify" "verify-promotion-bundle.sh" "Verify a signed promotion bundle";
          promotion-seal = repoApp "marketplace-promotion-seal" "seal-promotion-bundle.sh" "Seal and sign a promotion bundle";
        };

        devShells = {
          default = pkgs.mkShell {
            name = "mycelix-marketplace";
            packages = [ nodejs pkgs.nodePackages.pnpm pkgs.nodePackages.typescript ];
            shellHook = ''
              export NPM_CONFIG_PREFIX="$HOME/.npm-global"
              export PATH="$NPM_CONFIG_PREFIX/bin:$PATH"
              echo "Entered Mycelix frontend development shell."
            '';
          };
          promotion = pkgs.mkShell {
            name = "mycelix-marketplace-promotion";
            packages = promotionPackages;
            shellHook = ''
              export RUSTUP_HOME="${RUSTUP_HOME:-${XDG_CACHE_HOME:-$HOME/.cache}/mycelix-promotion/rustup}"
              export CARGO_HOME="${CARGO_HOME:-${XDG_CACHE_HOME:-$HOME/.cache}/mycelix-promotion/cargo}"
              export PATH="$CARGO_HOME/bin:$PATH"
              echo "Mycelix promotion shell"
              echo "Run: rustup toolchain install 1.94.0 --profile minimal --component rustfmt,clippy --target wasm32-unknown-unknown"
              echo "Then: nix run .#promotion-check"
            '';
          };
        };

        formatter = pkgs.nixpkgs-fmt;
      });
}
