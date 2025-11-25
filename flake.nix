{
  description = "Flake for Ki Editor (Bevy/Wayland)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crane.url = "github:ipetkov/crane";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      crane,
      flake-utils,
      rust-overlay,
      ...
    }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" ] (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ (import rust-overlay) ];
        };

        # 1. Rust Toolchain
        # Rapier and Bevy benefit from the latest stable compiler.
        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          extensions = [
            "rust-src"
            "rust-analyzer"
          ];
        };

        craneLib = (crane.mkLib pkgs).overrideToolchain rustToolchain;

        # 2. Runtime Dependencies (Dynamic Libraries)
        # These must be available at runtime for Winit (Windowing), Bevy (Rendering), and Alsa (Sound).
        buildInputs = with pkgs; [
          udev
          alsa-lib
          vulkan-loader

          # Wayland specific
          wayland
          wayland-scanner
          wayland-protocols
          libxkbcommon

          # X11 libs (Required by Winit for context creation/fallback, even on Wayland)
          xorg.libX11
          xorg.libXcursor
          xorg.libXrandr
          xorg.libXi
        ];

        # 3. Build-time Dependencies
        nativeBuildInputs = with pkgs; [
          pkg-config
          mold # Fast linker, significantly speeds up incremental Bevy/Rapier builds
          clang
        ];

        # Common arguments
        commonArgs = {
          src = craneLib.cleanCargoSource ./.;
          strictDeps = true;
          inherit buildInputs nativeBuildInputs;

          # Use Mold linker
          CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_LINKER = "clang";
          CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUSTFLAGS = [
            "-C"
            "link-arg=-fuse-ld=mold"
            # Optional: Uncomment to optimize physics for your specific CPU
            # "-C" "target-cpu=native"
          ];
        };

        # Build the package
        ki-editor = craneLib.buildPackage (
          commonArgs
          // {
            # Disable tests during build as they often require a GPU/Window system
            doCheck = false;
          }
        );

      in
      {
        packages.default = ki-editor;

        devShells.default = craneLib.devShell {
          inputsFrom = [ ki-editor ];

          packages = with pkgs; [
            # Dev tools
            cargo-watch
            cargo-nextest
            rust-analyzer

            # Debugging tools for Graphics/Wayland
            vulkan-tools
            wayland-utils
          ];

          # 4. Environment Setup
          # Critical for NixOS: Allows the binary to find Vulkan drivers and Wayland libs.
          shellHook = ''
            export LD_LIBRARY_PATH=${pkgs.lib.makeLibraryPath buildInputs}:$LD_LIBRARY_PATH

            # Force Winit to use Wayland (remove if you want XWayland fallback)
            export WINIT_UNIX_BACKEND=wayland

            echo "Ki Editor Dev Environment (Wayland/Rapier3D) Ready."
          '';
        };
      }
    );
}
