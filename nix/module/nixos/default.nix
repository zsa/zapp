{
  config,
  lib,
  ...
}:
let
  cfg = config.programs.zapp;
in
{
  imports = [ ../common.nix ];
  config = lib.mkIf cfg.enable {
    environment.systemPackages = [ cfg.package ];
    services.udev.packages = [ cfg.package ];
  };
}
