## Modules

The flake exposes two modules:

- `zapp.nixosModules.default` (for NixOS)
- `zapp.darwinModules.default` (for nix-darwin)

Each are called from their respective sections in the main flake.nix. Both 
modules source from nix/modules/common.nix, and only differ in how they handle
the module configuration: nixos registers a package with services.udev, while
nix-darwin does not.
