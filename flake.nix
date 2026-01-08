{
  description = "Unified Dev Environment (Android Cross-Compile + Native Wayland)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      rust-overlay,
      ...
    }:
    flake-utils.lib.eachSystem [ "x86_64-linux" ] (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ (import rust-overlay) ];
          config = {
            allowUnfree = true;
            android_sdk.accept_license = true;
          };
        };

        # 1. Rust Toolchain (Supports both Host and Android)
        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          extensions = [
            "rust-src"
            "rust-analyzer"
          ];
          targets = [ "aarch64-linux-android" ]; # Host target is included by default
        };

        # 2. Android SDK & NDK
        androidComposition = pkgs.androidenv.composeAndroidPackages {
          includeNDK = true;
          ndkVersions = [ "26.1.10909125" ];
          platformVersions = [ "34" ];
          buildToolsVersions = [ "34.0.0" ];
        };
        androidSdk = androidComposition.androidsdk;

        # 3. Native Linux Libraries (Required for 'cargo run')
        # These are used when compiling for your PC, but ignored when compiling for Android.
        hostLibs = with pkgs; [
          udev
          alsa-lib
          vulkan-loader

          # Wayland
          wayland
          wayland-scanner
          wayland-protocols
          libxkbcommon

          # X11 (For Winit fallback/compat)
          xorg.libX11
          xorg.libXcursor
          xorg.libXrandr
          xorg.libXi
        ];

      in
      {
        devShells.default = pkgs.mkShell {
          # Tools available in the shell
          nativeBuildInputs = [
            rustToolchain
            pkgs.pkg-config
            androidSdk
          ];

          # Libraries available for linking (Native only)
          buildInputs = hostLibs;

          shellHook = ''
            # --- Android Configuration ---
            export ANDROID_SDK_ROOT="${androidSdk}/libexec/android-sdk"
            export ANDROID_NDK_ROOT="$ANDROID_SDK_ROOT/ndk-bundle"
            NDK_TOOLCHAIN="$ANDROID_NDK_ROOT/toolchains/llvm/prebuilt/linux-x86_64/bin"
            ANDROID_API_LEVEL=24

            # Compilers for Android Target
            export CC_aarch64_linux_android="$NDK_TOOLCHAIN/aarch64-linux-android''${ANDROID_API_LEVEL}-clang"
            export CXX_aarch64_linux_android="$NDK_TOOLCHAIN/aarch64-linux-android''${ANDROID_API_LEVEL}-clang++"
            export AR_aarch64_linux_android="$NDK_TOOLCHAIN/llvm-ar"
            export CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER="$CC_aarch64_linux_android"

            # --- Native Linux Configuration ---
            # Needed so 'cargo run' can find Vulkan/Wayland at runtime
            export LD_LIBRARY_PATH=${pkgs.lib.makeLibraryPath hostLibs}:$LD_LIBRARY_PATH
            export WINIT_UNIX_BACKEND=wayland

            echo "Environment Ready."
            echo "  > Android Build: cargo build --target aarch64-linux-android"
            echo "  > Native Run:    cargo run"
          '';
        };
      }
    );
}
