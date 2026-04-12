# ds-emulator

A Nintendo DS emulator written in C++20, targeting Pokemon HeartGold / SoulSilver
primarily with full compatibility goals for all Gen 4 and Gen 5 Pokemon DS titles.

First-class build support for Apple Silicon macOS. Portable to Linux and Windows.

## Build & Run

```bash
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make
./ds_emulator <rom.nds> --scale 3
```

Apple Silicon optimized release build:

```bash
mkdir -p build && cd build && cmake .. -DDS_TARGET_APPLE_SILICON=ON -DCMAKE_BUILD_TYPE=Release && make
```

## Documentation

- **Design spec:** `docs/superpowers/specs/2026-04-12-nds-emulator-design.md`
- **Implementation plans:** `docs/superpowers/plans/`
- **Project conventions:** `CLAUDE.md`

## Status

Phase 0 — Scaffolding.
