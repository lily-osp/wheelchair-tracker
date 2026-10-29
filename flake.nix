# Development environment for the ESP32-C3 tracker firmware.
#
#   nix develop            # toolchain (pinned by firmware_rs/rust-toolchain.toml), espflash
#   nix run .#flash        # build + flash + monitor the board
#   nix run .#monitor      # monitor only
#
# Rust is pinned by firmware_rs/rust-toolchain.toml via rust-overlay, so every
# developer gets the same rustc with the riscv32imc target preinstalled.
{
  description = "wheelchair-tracker: ESP32-C3 + SIM7600 Rust firmware";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, rust-overlay }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      forAll = f: builtins.listToAttrs
        (builtins.map (s: { name = s; value = f s; }) systems);
    in
    {
      devShells = forAll (system:
        let
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ (import rust-overlay) ];
          };
          toolchain = pkgs.rust-bin.fromRustupToolchainFile
            ./firmware_rs/rust-toolchain.toml;
        in
        {
          default = pkgs.mkShell {
            packages = [
              toolchain
              pkgs.espflash
              pkgs.rust-analyzer
            ];
            env.RUST_SRC_PATH =
              "${toolchain}/lib/rustlib/src/rust/library/src";
          };
        });

      apps = forAll (system:
        let
          pkgs = import nixpkgs { inherit system; };
          fw = "firmware_rs/target/riscv32imc-unknown-none-elf/debug/firmware_rs";
          cargo = "cargo build --manifest-path firmware_rs/Cargo.toml";
        in
        {
          flash = {
            type = "app";
            program = toString (pkgs.writeShellScript "flash" ''
              set -e
              ${cargo}
              exec ${pkgs.espflash}/bin/espflash flash --chip esp32c3 --monitor ${fw}
            '');
          };
          monitor = {
            type = "app";
            program = toString (pkgs.writeShellScript "monitor" ''
              exec ${pkgs.espflash}/bin/espflash monitor --chip esp32c3 /dev/ttyACM0
            '');
          };
        });
    };
}
