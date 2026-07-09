{
  lib,
  ...
}:
{
  options.programs.zapp = {
    enable = lib.mkEnableOption "zapp, a CLI tool for flashing ZSA keyboards";

    package = lib.mkOption {
      type = lib.types.package;
      description = "The zapp package to use.";
    };
  };
}
