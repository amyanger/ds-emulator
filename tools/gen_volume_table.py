#!/usr/bin/env python3
"""Generate the ARM7 BIOS VolumeTable (SWI 0x1C GetVolumeTable).

Usage: python3 tools/gen_volume_table.py > /tmp/out.txt

Reproduces the 724-byte VolumeTable from the four-segment piecewise-log
shape documented in `docs/specs/2026-04-19-arm7-core-phase1-slice3h-design.md`
§4.7. Architecture rule #10 (no BIOS dumps) -- see CLAUDE.md. Do not paste
values from any BIOS reconstruction (melonDS freebios included); re-run this
script to regenerate.

Python version: Python 3.8+. stdlib only.

Output format: Byte-for-byte pasteable into
`src/cpu/arm7/bios/bios7_tables.cpp`; do not modify the format.
"""

import math
import sys

# Four piecewise-log segments. `start`/`end` are inclusive byte values; each
# segment contributes `length` consecutive bytes to the output array, in
# order. Offsets are derived cumulatively: 0, 189, 378, 515.
SEGMENTS = [
    # (start, end, length, shape)
    (0x00, 0x7F, 189, "log_floor_from_zero"),  # indices 0..0xBC
    (0x20, 0x7E, 189, "log_geom"),             # indices 0xBD..0x179
    (0x40, 0x7E, 137, "log_geom"),             # indices 0x17A..0x202
    (0x40, 0x7F, 209, "log_geom"),             # indices 0x203..0x2D3
]

TOTAL = 0x2D4  # 724


def compute_segment(start: int, end: int, length: int, shape: str) -> list:
    """Compute a single segment's `length` bytes per `shape`."""
    out = [0] * length
    if shape == "log_floor_from_zero":
        # start must be 0 here; interior follows a log curve rising to `end`.
        for i in range(length):
            v = end * (math.log(1 + i) / math.log(1 + (length - 1)))
            out[i] = int(math.floor(v + 0.5))
    elif shape == "log_geom":
        # Geometric interpolation between start and end (start > 0).
        for i in range(length):
            if length == 1:
                v = float(end)
            else:
                v = start * (end / start) ** (i / (length - 1))
            out[i] = int(math.floor(v + 0.5))
    else:
        raise ValueError(f"unknown shape: {shape}")
    # Lock endpoints structurally, regardless of float drift.
    out[0] = start
    out[length - 1] = end
    return out


def build_table() -> list:
    table = []
    for start, end, length, shape in SEGMENTS:
        table.extend(compute_segment(start, end, length, shape))
    return table


def main() -> int:
    table = build_table()

    # Hard asserts -- fail loudly if the shape drifts.
    assert len(table) == TOTAL, f"len={len(table)} expected {TOTAL}"
    assert table[0x000] == 0x00
    assert table[0x0BC] == 0x7F
    assert table[0x0BD] == 0x20
    assert table[0x179] == 0x7E
    assert table[0x17A] == 0x40
    assert table[0x202] == 0x7E
    assert table[0x203] == 0x40
    assert table[0x2D3] == 0x7F
    for i, b in enumerate(table):
        assert 0 <= b <= 0x7F, f"byte[{i}]={b:#x} out of range"

    # Emit the C++ block. 45 rows of 16 + 1 row of 4 = 724.
    out = sys.stdout
    out.write(
        "// VolumeTable: SWI 0x1C GetVolumeTable. 724 bytes produced once by\n"
        "// tools/gen_volume_table.py from the four-segment piecewise-log shape\n"
        "// documented in spec §4.7. The script is committed alongside these bytes;\n"
        "// re-running it must reproduce this array exactly.\n"
        "// Do not regenerate from a BIOS dump. To change these values, modify the\n"
        "// script's segment parameterization and re-run — never paste bytes from\n"
        "// any BIOS reconstruction (melonDS freebios included).\n"
        "// See CLAUDE.md architecture rule #10.\n"
    )
    out.write("// clang-format off\n")
    out.write("constexpr std::array<u8, kVolumeTableSize> kVolumeTable = {\n")

    # 45 full rows of 16 bytes.
    for row in range(45):
        base = row * 16
        cells = ", ".join(f"0x{table[base + j]:02X}" for j in range(16))
        out.write(f"    {cells},\n")
    # Final row of 4 bytes (indices 720..723 = 0x2D0..0x2D3), no trailing
    # comma on the very last element.
    final = table[720:724]
    cells = ", ".join(f"0x{b:02X}" for b in final)
    out.write(f"    {cells}\n")

    out.write("};\n")
    out.write("// clang-format on\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
