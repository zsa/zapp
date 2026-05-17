{
  config,
  lib,
  pkgs,
  ...
}:
let
  cfg = config.programs.zapp;
in
{
  options.programs.zapp = {
    enable = lib.mkEnableOption "zapp, a CLI tool for flashing ZSA keyboards";

    package = lib.mkOption {
      type = lib.types.package;
      description = "The zapp package to use.";
    };
  };

  config = lib.mkIf cfg.enable {
    environment.systemPackages = [ cfg.package ];
    services.udev.packages = [ cfg.package ];
  };
}
