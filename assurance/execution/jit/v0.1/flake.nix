{
  description = "Mycelix JIT qualification execution harness v0.1";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in {
      checks.${system}.jit-1b-vm = import ./vm/jit-1b-vm.nix {
        inherit pkgs;
        jitRoot = ./.;
      };
    };
}
