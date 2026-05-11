{
  pkgs ? import <nixpkgs> { },
}:
let
  zapp = import ./. { inherit pkgs; };
in
pkgs.mkShell {
  inputsFrom = [ zapp ];
  packages = [
    pkgs.cargo
    pkgs.rustc
    pkgs.rustfmt
    pkgs.clippy
  ];
}
