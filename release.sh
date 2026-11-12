#!/usr/bin/env bash
# Builds the release executables for Linux (glibc 2.35+) and Windows x64.
# Requirements: docker, the x86_64-pc-windows-gnu Rust target and mingw-w64.
set -euo pipefail
cd "$(dirname "$0")"
VERSION=$(grep -m1 '^version' Cargo.toml | cut -d'"' -f2)
OUT=dist/v$VERSION
docker build -q -t actvib-linux-build -f Dockerfile.linux .
docker run --rm -v "$PWD":/src -v "$HOME/.cargo/registry":/root/.cargo/registry actvib-linux-build \
    sh -c "cargo build --release --target-dir target/linux-compat && chown -R $(id -u):$(id -g) target/linux-compat"
cargo build --release --target x86_64-pc-windows-gnu
mkdir -p "$OUT"
cp target/linux-compat/release/actvib "$OUT/ActVib-$VERSION-linux-x86_64"
cp target/x86_64-pc-windows-gnu/release/actvib.exe "$OUT/ActVib-$VERSION-windows-x64.exe"
cp LEIAME-turma.txt "$OUT/LEIAME.txt"
(cd "$OUT" && sha256sum ActVib-* > SHA256SUMS.txt)
ls -la "$OUT"
