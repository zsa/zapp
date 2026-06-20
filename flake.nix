{
  description = "Zapp - CLI tool for flashing ZSA keyboards";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs =
    { self, nixpkgs }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
      ];
      forAllSystems = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in
    {
      overlays.default = import ./nix/overlay.nix;

      packages = forAllSystems (pkgs: rec {
        zapp = pkgs.callPackage ./nix/package.nix { };
        default = zapp;
      });

      devShells = forAllSystems (pkgs: {
        default = pkgs.mkShell {
          inputsFrom = [ self.packages.${pkgs.system}.zapp ];
          packages = [
            pkgs.cargo
            pkgs.rustc
            pkgs.rustfmt
            pkgs.clippy
            pkgs.nixfmt
          ];
        };
      });

      formatter = forAllSystems (pkgs: pkgs.nixfmt);

      nixosModules.default =
        { pkgs, ... }:
        {
          imports = [ ./nix/module/nixos ];
          programs.zapp.package = nixpkgs.lib.mkDefault self.packages.${pkgs.system}.default;
        };

      darwinModules.default =
        { pkgs, ... }:
        {
          imports = [ ./nix/module/darwin ];
          programs.zapp.package = nixpkgs.lib.mkDefault self.packages.${pkgs.system}.default;
        };
    };
}
