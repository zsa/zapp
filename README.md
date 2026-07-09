# ⚡ Zapp

A cross-platform CLI tool for flashing [ZSA](https://www.zsa.io) keyboards.

## Supported keyboards

- Voyager
- Moonlander MK1 (rev A and rev B)
- Ergodox EZ STM32/Teensy (including Original, Shine, and Glow variants)
- Halfmoon
- Planck EZ (including Standard and Glow variants)

## Installation

You can download the latest version of zapp from the [releases](https://github.com/zsa/zapp/releases) page.

### macOS with Homebrew

```sh
brew install zapp
```

nix-darwin is also supported. See the [Nix flakes](#flakes) section below for details.

### openSUSE

```sh
zypper ar obs://hardware obs-hardware
zypper in zapp
```

### Arch Linux

Using an AUR helper like `yay` or `paru` *(handles udev rules)*:

```sh
paru -Sy zsa-zapp-git   # build from latest commit
```

### From source

```sh
cargo install --path zapp
```

### As pre-built binaries from GitHub Releases with mise

```sh
mise use -g github:zsa/zapp
```

### Nix

#### Flakes

Run without installing:

```sh
nix run github:zsa/zapp -- flash firmware.bin
```

Install into a profile:

```sh
nix profile install github:zsa/zapp
```

On NixOS, the flake exposes a module that installs the binary and registers the udev rules system-wide:

```nix
{
  inputs.zapp.url = "github:zsa/zapp";

  outputs = { self, nixpkgs, zapp, ... }: {
    nixosConfigurations.my-host = nixpkgs.lib.nixosSystem {
      system = "x86_64-linux";
      modules = [
        zapp.nixosModules.default
        { programs.zapp.enable = true; }
      ];
    };
  };
}
```

On nix-darwin, the flake exposes a module that installs the binary:

```nix
{
  inputs.zapp.url = "github:zsa/zapp";

  outputs = { self, nixpkgs, zapp, ... }: {
    darwinConfigurations.my-host = nix-darwin.lib.darwinSystem {
      system = "aarch64-darwin";
      modules = [
        zapp.darwinModules.default
        { programs.zapp.enable = true; }
      ];
    };
  };
}
```

(Adjust nix-darwin.lib.darwinSystem to match your nix-darwin input.)

An overlay is also available (`zapp.overlays.default`) which adds `pkgs.zapp` to nixpkgs.

A `nix develop` shell with the Rust toolchain and `nixfmt` is provided for hacking on the flake itself.

#### Without flakes

The repository ships a top-level `default.nix` and `shell.nix`, so `nix-build` / `nix-shell` work directly against a checkout:

```sh
nix-build      # builds the package, symlinked at ./result
nix-shell      # dev shell with the Rust toolchain
```

To consume from another non-flake config, import the overlay or the package directly:

```nix
# configuration.nix
{ pkgs, ... }:
let
  zapp = import (builtins.fetchTarball "https://github.com/zsa/zapp/archive/main.tar.gz") { inherit pkgs; };
in {
  environment.systemPackages = [ zapp ];
  services.udev.packages = [ zapp ];
}
```

Or pin via `niv` / `npins` and import `nix/overlay.nix` into your nixpkgs config.

### Linux: udev rules

On Linux, you need udev rules to access USB devices without root:

```sh
sudo cp udev/50-zsa.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Usage

### Flash from a local file

```sh
zapp flash firmware.bin
zapp flash firmware.hex
```

### Flash from an Oryx URL

Paste your layout URL directly from [Oryx](https://configure.zsa.io) — zapp will download and flash the firmware for you:

```sh
zapp flash https://configure.zsa.io/voyager/layouts/oBvbQ/latest
zapp flash https://configure.zsa.io/moonlander/layouts/AbCdE/abc123
```

All standard Oryx URL forms are supported:

| URL form                            | Behavior                       |
| ----------------------------------- | ------------------------------ |
| `.../layouts/:layoutId`             | Fetches the latest revision    |
| `.../layouts/:layoutId/latest`      | Fetches the latest revision    |
| `.../layouts/:layoutId/latest/0`    | Fetches the latest revision    |
| `.../layouts/:layoutId/:revisionId` | Fetches that specific revision |

### Update your current layout

If your keyboard is already running a firmware built with Oryx, `zapp update` will check for a newer revision and flash it:

```sh
zapp update
```

This reads the layout and revision from the keyboard's USB serial number, checks the Oryx API, and flashes automatically if an update is available.

> **Note:** This only works if your keyboard is currently flashed with firmware from Oryx and is not in bootloader mode.

### Flashing process

After loading or downloading firmware, zapp waits for the keyboard to enter bootloader mode. Reset your keyboard (e.g., press the reset button or use a key combo), and zapp will detect it and flash automatically:

```
Firmware loaded: Voyager (STM32) (51200 bytes)
_ Waiting for keyboard in bootloader mode...
⚡ ████████████████████████████████████████ 100% Done!
```

## Firmware formats

| Format     | Extension | Keyboards                                             |
| ---------- | --------- | ----------------------------------------------------- |
| DFU binary | `.bin`    | Voyager, Moonlander, Planck EZ, Ergodox EZ (Ignition) |
| Intel HEX  | `.hex`    | Ergodox EZ (original, HALFKAY)                        |

Dual-firmware files (e.g., Moonlander rev A + rev B, Voyager STM32 + GD32) are detected and handled automatically.

## Project structure

```
zapp/           CLI binary (clap, indicatif, reqwest)
zapp-core/      Core library
  device/       USB device detection and identification
  firmware/     Firmware parsing (DFU binary, Intel HEX)
  flash/        Flashing protocols (STM32 DFU, HALFKAY)
udev/           Linux udev rules
```

## License

Copyright ZSA Technology Labs, Inc. See [LICENSE.md](LICENSE.md) for details.
