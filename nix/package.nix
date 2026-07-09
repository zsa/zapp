{
  lib,
  rustPlatform,
  pkg-config,
  libusb1,
  stdenv,
}:
let
  fs = lib.fileset;
  src = fs.toSource {
    root = ../.;
    fileset = fs.unions (
      [
        ../Cargo.toml
        ../Cargo.lock
        ../zapp
        ../zapp-core
        ../zapp-oryx
      ]
      ++ lib.optional (!stdenv.isDarwin) ../udev
    );
  };
  cargoToml = builtins.fromTOML (builtins.readFile ../zapp/Cargo.toml);
in
rustPlatform.buildRustPackage {
  pname = cargoToml.package.name;
  version = cargoToml.package.version;
  inherit src;
  cargoLock.lockFile = ../Cargo.lock;

  nativeBuildInputs = [ pkg-config ];
  buildInputs = [ libusb1 ];

  postInstall = lib.optionalString stdenv.isLinux ''
    install -Dm644 udev/50-zsa.rules $out/lib/udev/rules.d/50-zsa.rules
  '';

  meta = {
    description = "CLI tool for flashing ZSA keyboards";
    homepage = "https://github.com/zsa/zapp";
    license = lib.licenses.mit;
    mainProgram = "zapp";
    platforms = lib.platforms.linux ++ lib.platforms.darwin;
  };
}
