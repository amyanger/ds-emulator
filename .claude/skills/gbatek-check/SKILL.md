---
name: gbatek-check
description: Verify DS hardware implementation against GBATEK. Use before implementing or modifying any CPU instruction, I/O register, VRAM mapping, scheduler timing, SPU channel behavior, 3D engine command, or cart protocol — anything where "how the hardware actually behaves" is the question. Also use when reviewing existing code for a bug that smells like a hardware-behavior miss.
---

# GBATEK Hardware Check

**Your #1 rule for this project: never implement DS hardware from memory.** The DS has too many non-obvious quirks, and plausible-looking code that doesn't match the hardware is the single biggest source of emulator bugs. This skill forces you to cross-reference GBATEK before and during implementation.

## When to invoke this skill

Run this skill when you are about to:
- Decode or execute a CPU instruction (ARM9 or ARM7)
- Read or write an I/O register for the first time
- Add or modify a scheduler event kind or its latency
- Touch VRAM bank mapping (`VRAMCNT_*`)
- Add a DMA, IRQ, IPC, timer, cart, or SPU behavior
- Implement any 3D geometry command or rasterizer step
- Debug a divergence where the symptom looks like "I think I modeled this wrong"

## Steps

1. **Identify the exact subsystem or register.** Examples: `LDR (immediate)`, `VRAMCNT_A`, `GXSTAT`, `SOUNDCNT`, `KEY1 cart encryption`, `SWI 0x09`. Be specific — "the PPU" is too broad.

2. **Fetch the authoritative spec.** Use the `firecrawl` MCP (already installed) to scrape the relevant GBATEK section. Primary URLs:
   - https://problemkaputt.de/gbatek.htm — official GBATEK
   - https://mgba-emu.github.io/gbatek/ — searchable Markdown mirror (usually easier)

   Scrape the smallest section that covers the behavior. Do not dump the whole page.

3. **Extract the concrete hardware facts.** For each behavior, write down in plain English:
   - What bit widths, masks, and register layouts are involved?
   - What are the side-effects (flag updates, IRQ assertion, IF bits, etc.)?
   - What are the timing characteristics (how many cycles, what triggers the event)?
   - What are the documented edge cases (wraparound, saturation, "prohibited" values, undefined-but-consistent behavior)?
   - Are there ARM9-vs-ARM7 differences?

4. **Compare against the current implementation.** If code already exists for this subsystem, read it and diff it against the spec. List discrepancies explicitly — do not say "looks correct" without a per-item check.

5. **Report.** Produce a short report with three sections:
   - **Spec summary** — the hardware facts from step 3.
   - **Discrepancies** — any mismatch between current code and spec. If none, say "none found — matches spec."
   - **Implementation notes** — anything non-obvious that the implementer should keep in mind (e.g. "writing 1 to IF clears that bit — opposite of most registers").

## Red flags that mean STOP and run this skill

- You're typing bit masks from memory.
- You're about to write `0x...` for a register address you didn't just look up.
- You're implementing an instruction and skipping over "the weird edge case" because you'll "handle it later."
- You think you remember how VRAM bank N maps at MST=M.
- You're writing SPU / ADPCM code without the prediction table in front of you.

## What this skill is NOT

It is not a replacement for test ROMs. After implementing, you still need to run `armwrestler-nds`, `gbatek-armtest`, or a dedicated unit test to prove correctness. This skill prevents the "plausible but wrong" class of bug; test ROMs catch what's left.
