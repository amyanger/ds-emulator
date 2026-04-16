---
name: run-arm-tests
description: Build the ds-emulator and run the full CTest unit suite, reporting pass/fail clearly. Use before starting a new implementation task (to confirm a green baseline), after finishing a task (to confirm no regressions), and whenever you want a quick "is everything still working?" check. Also use before committing.
---

# Run ARM / CTest Unit Tests

Quick, reliable way to verify the current state of the emulator's test suite. Runs the full CTest pass and surfaces any failures.

## When to invoke

- **Before starting a task** — confirm the baseline is green. If tests are already failing, the new work will be blamed for pre-existing breakage.
- **After finishing a task** — confirm no regressions before the `/simplify` → `quality-reviewer` → commit sequence.
- **Before committing anything** — part of verification-before-completion.
- **When debugging a divergence** — isolate which test first fails.

## Steps

1. **Ensure the build directory exists and is configured.**
   ```bash
   cd /Users/arjunmyanger/Documents/Dev/ds-emulator
   if [[ ! -d build ]]; then
     mkdir build
   fi
   cd build
   if [[ ! -f CMakeCache.txt ]]; then
     cmake .. -DCMAKE_BUILD_TYPE=Debug
   fi
   ```

2. **Build.** Use `make` (or `cmake --build .`) and watch for compile errors via the Monitor tool:
   ```bash
   cmake --build . 2>&1 | grep --line-buffered -E "error:|warning:|undefined reference|fatal"
   ```
   Stop and report immediately if the build fails — do not run tests against stale binaries.

3. **Run CTest.** Always use `--output-on-failure` so a failing test shows its stderr inline:
   ```bash
   ctest --output-on-failure
   ```

4. **Report.** Format the report as:
   - **Build:** clean / warnings / failed
   - **Tests passed:** N / total
   - **Failures:** list each failing test name and the first error line from its output, or "none"
   - **Next action:** one sentence — green baseline OK to proceed, or "investigate <test> before continuing"

## Caveats

- **Debug build is mandatory for test runs.** Tests use `assert()`, which is a no-op in Release. There's an existing memory note about this: until Phase 1 fixes it, never run the test suite against a Release build and claim it passed.
- **Do not skip tests.** If a test is slow, fix the test, don't exclude it.
- **Do not cache results.** Always actually re-run — file-timestamp-based assumptions have bitten this project before.

## Future expansion (once Phase 6+ lands)

When the headless regression harness exists (`./ds_emulator --headless --frames N --dump-hash out.txt`), extend this skill to also run the recorded-input regression suite and diff framebuffer hashes. Not yet applicable in Phase 1.
