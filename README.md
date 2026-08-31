# Exhaust Fabricator

**A Blender add-on (extension) for parametric automotive exhaust fabrication.**

Exhaust Fabricator lets you design and model automotive exhaust systems directly in Blender as editable, fabrication-oriented parts rather than freeform meshes or generic curve sweeps. It's built specifically for exhaust/header work, not general industrial piping.

## What it does

- **Routes** — straight tube and mandrel-bend segments, built up piece by piece, with configurable OD/wall thickness, end treatments (plain, expanded, reduced, slip-socket), and bolt-on connection hardware (V-band, round weld flange, 2-bolt and 3-bolt automotive flat flanges)
- **Pie-cut bends** — fabrication-style bends made from straight mitered tube sections welded together, as an alternative to a smooth mandrel bend
- **Reducers/expanders** — simple conical transitions between two pipe diameters
- **Y-pipes, X-pipes, H-pipes** — junction/merge components with true communicating crossovers and shared saddle merges, in several topology styles (swept, classic, parallel, tangent/side-entry, custom)
- **Collectors** — radially symmetric N-to-1 merge collectors (4-into-1, 6-into-1, etc.) with configurable outlet fabrication and connection hardware
- **Headers** — the flagship feature: a coordinated set of primary pipes (one per exhaust port) sharing one cylinder-head flange, with:
  - live equal-length analysis across all primaries
  - a shared head flange generated automatically from the primary layout
  - an assisted routing solver that automatically generates a collision-free, length-matched path from each primary's start to its assigned collector inlet — including a "Complete to Collector" mode that keeps whatever segments you've drawn by hand and auto-routes only the remainder
  - optional primary-to-primary collision avoidance, keep-out geometry (route around obstacles like an oil pan or frame rail), and editable guide-spline routing hints

## Requirements

Blender 4.2 or newer. Install the latest release zip from this repo as a Blender extension (Edit → Preferences → Get Extensions → Install from Disk, or drag-and-drop onto the Blender window).

## Status

The core Route/Collector/junction/reducer modeling is stable. The assisted-routing / keep-out / guide-spline Header workflow (see the version history below and `Exhaust_Fabricator_Handoff_v0.14.5.md` for full architecture notes) is still experimental and under active development.

---

# Version history

## Header primary interface cleanup

Header-managed primaries no longer expose or generate individual **Start End / Start Connection** or **Finish End / Finish Connection** fabrication.

A Header primary is now treated as the tube between two managed interfaces:

- upstream: the Header's single shared head flange
- downstream: its assigned Collector inlet

The old Route properties remain stored for file compatibility, but the geometry resolver forces both ends to Plain / None while the Route belongs to a Header.

## Collector outlet fabrication

The Collector now owns the downstream end-treatment and connection-hardware controls:

- Plain
- Expanded / Swaged
- Reduced / Necked
- Slip Socket (Female)

and:

- None
- V-Band Flange
- Round Weld Flange
- 2-Bolt Flat Flange
- 3-Bolt Flat Flange

The outlet treatment/hardware is appended to the Collector's existing final round outlet ring. The original cut-face cap is removed and its boundary vertices are reused as the first ring of the fabrication, keeping the outlet connected to the Collector mesh rather than placing loose hardware over it.

Collector end-connector metadata is moved to the actual exposed end/connection face so subsequent routing snaps to the real outlet location.

## Collector-to-Route workflow

Two explicit workflows were added for normal Exhaust Routes:

### Add Route from Outlet

Select a Collector and click **Add Route from Outlet** in the Collector panel. A normal Route is created and aligned to the Collector's exposed outlet connector. If the Collector has connection hardware, matching start-side hardware is copied to the new Route so the two exposed connection faces mate.

### Snap Start to Selected Collector

A normal Route now has **Collector Attachment > Snap Start to Selected Collector**. Select the Route plus one Collector, keep the Route active, and click the button. The Route's actual start connector (including any start-end treatment/hardware) is aligned to the Collector's actual exposed outlet connector.

The older generic two-object connector snap remains available as well.

## Solver profiling (new in v0.14.6)

`Generate Active Primary` and `Generate All Primaries` now record a per-stage wall-clock breakdown of the assisted-routing solve: candidate generation, primary-vs-primary collision checks, keep-out mesh collision checks, obstacle-chain building, guide-spline sampling, per-order-attempt cost, and per-collector-phase-attempt cost.

This is purely diagnostic — it does not change solver behavior or output geometry. It exists to identify where the v0.14.x routing stack (keep-out solving, collector radial-phase search, guide-spline solving) is actually spending time before that time is optimized away, per the handoff's Step 2 ("profile solver performance").

Controls live under Header > Assisted Routing > **Solver Diagnostics — Experimental**:

- **Profile Solve Performance** (on by default): times the next Generate Active/All Primaries run.
- The captured report is shown inline (per-stage ms, % of total, call count, ms/call) and also printed to Blender's system console (Window > Toggle System Console on Windows) so long reports aren't clipped by panel width.

Note: some reported stages nest inside others (e.g. `order_attempt` includes `primary_solve`, which includes `candidate_generation`/`primary_collision_check`). Percentages are relative to total wall time, not a strict non-overlapping partition, so leaf-stage percentages can sum to more than 100% when a parent stage is also shown. This is called out in the profiling module docstring and in the panel itself.

## Primary-collision performance fix (new in v0.14.7)

The v0.14.6 profiling instrumentation was run against a real 4-primary Header (collisions + keep-outs + collector radial-phase search all enabled) and found the actual dominant cost was **not** keep-out mesh collision as the handoff's section 51 assumed — it was `primary_collision_check` (sibling-primary capsule-vs-capsule testing), at 82.7% of total solve time, roughly 30x more expensive per call than the keep-out mesh path despite testing simpler geometry. Root cause: `capsule_chain_clearance()` in `collision.py` did a full unaccelerated O(N×M) segment-pair sweep with no broad phase, unlike the keep-out path which already had a spatial-hash grid.

Two changes, both in the collision/search-time code only — final built geometry and the postbuild safety gates are untouched:

1. **Bounding-box broad phase**, both at the whole-chain level and per segment pair, before falling back to the exact segment/segment distance test. A pair is only ever skipped once its own inflated bounding boxes are proven disjoint, which is analytically only possible when that pair's true clearance is positive — so a real overlap can never be pruned away (see `capsule_chain_clearance`'s docstring in `collision.py` for the exact safety argument). New `capsule_chain_bounds()` lets a fixed sibling-primary obstacle's bounds be built once and reused across the thousands of candidates checked against it in one primary solve, instead of rebuilt every call.
2. **Decoupled the collision-search sample count from the final route's visual bend resolution.** The search's per-candidate capsule chain was previously built at `max(route_bend_resolution, quality_default)` segments per bend — meaning cranking up "Bend Segments" for a smoother final model also silently made every collision check during the *search* more expensive, even though the sagitta-based radius inflation already keeps a coarser proxy chain conservative at any sample count. Collision sampling is now `min(route_bend_resolution, quality_default)` instead, capped independently of the final geometry.

Correctness was verified with a 606-case standalone test (`test_capsule_clearance.py`, not shipped in the add-on zip) comparing the optimized function against a brute-force reference across random and boundary-straddling chain pairs, including real collisions — every genuine overlap matched exactly; only the reported *margin* on already-safe pairs can be a conservative overestimate (never wrong-signed).

Measured on the same 4-primary test scenario used to find the bottleneck (Blender 5.2.1, background mode):

| Operator | v0.14.6 | v0.14.7 | Speedup |
|---|---|---|---|
| Generate All Primaries (collisions+keep-outs+phase search) | 114.0 s | 22.8 s | **5.0x** |
| Generate Active Primary (guide-spline solve) | 18.7 s | 1.5 s | **12.4x** |
| `primary_collision_check` cost per call | 4.2–5.1 ms | 0.20–0.23 ms | **~21x** |

Both runs produced identical solver output (same primary order, same collector phase, same length error) before and after — this is a pure performance change, not a behavior change.

## Combinatorial search size (new in v0.14.8)

v0.14.7 fixed the per-call cost of primary-vs-primary collision checking, but the same 6-primary/NORMAL-quality scenario that motivated it still pegs a CPU for 20+ minutes when it never finds a valid layout, because the *volume* of work is a product of several independent knobs (phase samples × solve orders × primaries × the a/b/offset/clocking candidate grid per primary — see the handoff's section 48). Two changes, both in `operators.py`, target that volume directly rather than any single collision test:

### Orders To Compare (`solver_order_success_cap`, default 3)

`Generate All Primaries` tries several different primary solve orders (forward, reverse, cyclic, outside-in, center-out — up to 10 of them at NORMAL quality) and keeps the best-scoring complete one, because an early primary can otherwise hog the only clean routing corridor for a later one. Previously it always tried every generated order, even after several had already succeeded and could only change a low-priority tie-break, not whether the Header solves at all. It now stops once this many complete, collision-free orders have been found — mirroring the success-cap already used for collector radial-phase search.

### Max Search Time (`solver_max_solve_seconds`, default 60s)

A safety ceiling on the combined order/phase search for `Generate All Primaries`, so a Header with no valid solution doesn't search indefinitely. This is checked **only between whole order and phase attempts, never in the middle of one** — that's a deliberate choice, not a shortcut: an attempt that's most of the way through solving every primary is also close to a real solution, and killing it partway would trade a working (if slow) answer for a guaranteed "no solution found." (An earlier version of this checked between primaries within an order too, and testing showed exactly that failure mode — an order that would have succeeded on its own got cut short and turned into a false negative. It was reverted.) The practical consequence: total time is bounded by roughly this budget plus one attempt's own worst-case duration, so it's meant to be tuned together with Search Quality, which controls how long a single attempt can take. When the budget runs out, the search stops and reports the same "no solution found, previous routes restored" outcome as if every attempt had simply been tried and failed — never a partial or colliding commit.

Both are exposed under Header > Assisted Routing > **Search Budget**, visible whenever collision avoidance, keep-outs, or phase search are active.

### Measured

On the exact 6-primary/NORMAL/collisions+keepouts+phase-search scenario that ran 20+ minutes unfinished before v0.14.7: it now reliably finds a complete, collision-free solution in **65–106 seconds** (which order happens to succeed first varies slightly run to run — see note below — but every run tested found a valid answer well within a couple of minutes, a real result instead of an indefinite hang). With an artificially tight 5s budget on the same scenario, it correctly reports no-solution-found and restores the previous routes rather than running away, in about one order attempt's worth of overrun.

*Update (v0.14.9): the run-to-run variation noted above was investigated and turned out to be a false alarm — see below.*

## Investigating the apparent non-determinism (v0.14.9)

The note above, written while testing v0.14.8, turned out to be comparing two different in-progress edits of `operators.py`, not two runs of the same program: a mid-order per-primary deadline check had been added, then reverted after testing showed it caused a worse failure mode (see the "Max Search Time" section above), and the default search budget had also changed in between. Once that settled, four independent tests against the actual shipped v0.14.8 code all came back fully deterministic:

1. The identical 6-primary/NORMAL/collisions+keepouts+phase-search scenario, solved 10 times within one Blender process (5 on an unchanging Header, 5 with the scene rebuilt from scratch each time) — identical order, phase, and primary lengths every time.
2. `solve_primary_route()` in isolation (it has no `bpy` dependency), called dozens of times with fixed inputs across several separate Python process launches with confirmed different `PYTHONHASHSEED` values each time — bit-identical results regardless.
3. The full scenario launched as 10 genuinely separate `blender.exe` processes (FAST and NORMAL quality, profiling on and off) — identical results every time, including a SHA-256 fingerprint of the depsgraph-evaluated keep-out mesh's own vertex coordinates, ruling out Blender's mesh evaluation as a source too.

No non-determinism bug exists in the solver as shipped. This is a documentation correction only; v0.14.9's solver code is unchanged from v0.14.8.

## Keep-out collision cost (v0.14.10)

Profiling in v0.14.7-9 used a bare primitive cube (8 vertices, 12 triangles) as the test keep-out object, which was too trivial to expose the real cost of keep-out collision against the complex meshes the handoff actually describes (engine blocks, frame rails, transmissions — thousands of triangles). Re-tested against a 3072-triangle mesh (a heavily subdivided cube), which changes the picture completely: **keep-out collision jumps to 57% of total solve time** — it genuinely is the bottleneck the handoff originally suspected, just not visible with a trivial test object.

Two fixes shipped, one real win and one correct-but-not-the-bottleneck-here:

### 1. Keep-out obstacle built once per Generate call, not once per primary (real win)

`_collision_obstacles_for_primary()` was rebuilding each keep-out object's evaluated mesh, triangulation, and spatial-hash grid from scratch on *every* call — once per primary, per order attempt, per phase candidate (hundreds of times in one Generate All). None of that depends on which primary is being solved, only on the keep-out object's own geometry. It's now built once per Generate call, in world space, and each candidate's own (much smaller — a few dozen segments) capsule chain is transformed into world space instead, at collision-check time.

Measured on the 3072-triangle mesh: `obstacle_build_keepout` went from 282ms across 14 calls (once per primary solved) to 20ms across 1 call — confirmed to scale linearly with however many primaries get solved in one Generate All, so the saving is proportionally larger on bigger Headers (more primaries, more order/phase attempts) with complex keep-outs, even though it's a rounding error next to the 51-second total in this specific 4-primary test.

### 2. `_point_inside_triangle_mesh` given a spatial-hash broad phase (correct, but didn't move this benchmark)

This "is the tube embedded deep inside solid keep-out material" fallback check was ray-casting against *every* triangle in the mesh unconditionally, with no broad phase at all, and was being invoked for nearly every safely-clear capsule segment. It's now accelerated with a conservative grid-cell walk along the ray (proven correct: a triangle can only be pruned once its cell is proven un-crossed by the ray, and every truly-intersecting triangle's cell is guaranteed to be visited — see the docstring in `collision.py`), plus the escape ray itself was shortened from a `bbox_diagonal*2.5 + 1.0m` overshoot down to `bbox_diagonal*1.5` (a flat 1-meter margin dominates for small mechanical parts, and was making the *new* grid-walk's own cost scale badly along with it — both were part of the same fix). Verified against a 400-case brute-force comparison across 8 random mesh configurations, including near-surface boundary points: 0 mismatches.

This change is correct and safe, but `cProfile` showed it was never actually the bottleneck for the dense-mesh benchmark — the broad phase in the main surface-distance path already prunes candidate triangles down to about 1 per capsule segment on average, and that one exact triangle-distance computation (`segment_triangle_distance`, itself calling six sub-computations: a Möller–Trumbore intersection test, two closest-point-on-triangle solves, and three edge-segment-distance checks) is where the real cost lives. Speeding that up further would mean reworking a geometric primitive that other collision paths also depend on for correctness, which is a materially riskier change than the broad-phase/caching wins made so far — it's flagged here as the next concrete target, not attempted in this pass.

Both fixes were verified end-to-end against the existing 4-primary and 6-primary test scenarios: identical solver output (same order, phase, and lengths) before and after, on top of their own standalone correctness tests.

## segment_triangle_distance sped up (v0.14.11)

v0.14.10's `cProfile` pass identified this exact-math primitive itself — not any missing broad-phase pruning — as the true remaining bottleneck for complex keep-out meshes: the existing broad phase already narrows each capsule segment down to about one candidate triangle, and it's that *one* exact distance computation that's expensive. `segment_triangle_distance` runs six sub-computations per call (a Möller–Trumbore plane-intersection test, two closest-point-on-triangle solves for the segment's endpoints, and three edge-vs-segment distance checks for the triangle's three edges) — and each of those six was independently recomputing quantities that are identical across all six: the segment's own direction vector, and the triangle's two edge vectors from vertex A.

The fix is a pure algebraic refactor, not an algorithm change: compute the segment direction and triangle edge vectors **once** at the top of `segment_triangle_distance`, and pass them into new precomputed variants of the three sub-routines (`_closest_point_triangle_precomp`, `_segment_triangle_intersection_t_precomp`, `_segment_segment_distance_precomp`) instead of each one re-deriving them from scratch. Same six checks, same branches, same math — just without repeating identical Vector subtractions and dot products six times over.

Verified against the original (unshared) implementation across 3600 test cases — bulk random configurations plus deliberately adversarial ones (segments piercing the triangle, zero-length segments, degenerate near-collinear triangles, segments parallel to an edge, query points exactly on a vertex or edge): every case matched to floating-point tolerance, confirming this is a true refactor with no change in results.

Measured on the same 3072-triangle dense-mesh benchmark: `keepout_collision_check` dropped from 2.29ms to 1.85ms per call (~19% faster), and total solve time on that scenario went from 51.4s to 45.4s (~12% faster overall). More modest than the broad-phase/caching wins in v0.14.7 and v0.14.10 — this is squeezing a primitive that was already fairly efficient, not eliminating a missing optimization — but real, safe, and free: same result, less repeated arithmetic to get there.

Full pipeline re-verified against the standard 4-primary and 6-primary test scenarios: identical solver output (same order, phase, lengths) before and after.

## v0.14.12: an early-exit fix that shipped, and a density reduction that didn't

Two further changes were attempted together. Only one survived verification — the other is documented here specifically because it looked safe by every measure except the one that actually caught it, which is a real lesson about how to trust this kind of change going forward.

### Shipped: `segment_triangle_distance` given a proper early-exit contract

The function now accepts `early_exit_below`: once any of its six sub-checks proves the true distance is at most that value, it returns immediately instead of computing the rest — safe because every sub-check only ever finds a distance ≥ the true minimum, so one witness below a threshold already proves the true minimum is too. `capsule_chain_mesh_clearance` passes its own collision-radius requirement through, so a segment that clearly collides with a given triangle stops checking that triangle's remaining sub-tests immediately, matching this codebase's existing "sign matters, not exact severity once collision is proven" pattern from the v0.14.7 primary-collision fix.

The first version of this had a genuine bug, caught only by accident while investigating an unrelated regression (see below), not by its own dedicated test suite: it used the collision-radius requirement itself as the threshold, but the actual outer decision is `distance - requirement < -1e-10` — a *strictly smaller* bound. Passing the loose threshold let the function return an intermediate witness sitting in the `[-1e-10, 0)` gap on a triangle that only looked borderline-safe, before ever reaching a *later* triangle whose true distance was unambiguously inside the obstacle — silently downgrading a real collision to "not rejected." Fixed by passing `required - 1e-10`, which provably preserves the exact same accept/reject decision as computing the full six-check minimum every time (any witness below that stricter bound guarantees the true minimum is too). Verified via 2000 additional threshold-contract test cases plus the full pipeline scenarios.

### Reverted: reducing the coarse clocking-search grid

The idea: the coarse clocking grid multiplies the entire a/b/offset candidate grid, making it the single largest cost lever in `solve_primary_route`; halve it and let the existing local-refinement pass also fine-tune clocking (the same way it already fine-tunes a/b/offset) to compensate for the coarser initial sampling.

This looked solid under testing: 90+ synthetic random collision-avoidance scenarios across FAST/NORMAL/HIGH quality showed **zero** success-rate regressions and 1.2-1.9x speedups, with length-error deltas ranging from negligible (HIGH: <0.01mm) to modest (FAST: up to 21mm on an 0.85m target). It shipped to the full-pipeline test scenarios anyway — and immediately broke the standard 4-primary reference scenario, which had solved reliably through every prior version, with "no collision-free route found" for the first primary.

Root-caused down to: it was real, it reproduced on the untouched pre-change baseline vs. the changed code side by side, and disabling each individual piece (the density reduction alone, the early-exit change alone) didn't fix it — only reverting the clocking-refinement addition in full did. The exact mechanism inside the refinement/scoring interaction wasn't pinned down with full confidence in the time available, and shipping a change whose failure mode isn't fully understood is a worse trade than not shipping it, however good the synthetic numbers looked. Reverted in full; `solve_primary_route`'s clocking search is back to the exact v0.14.11 grid.

**The actual lesson, worth keeping**: this regression was *only* caught because the full-Blender-pipeline verification step was run at all -- and it was almost skipped by accident, because the test script's import path had gone stale after being copied across three prior version folders and was silently exercising the *previous* version's code the whole time this session, right up until the stale path was noticed and fixed. Once corrected, the very first real-pipeline run caught what 90+ well-designed synthetic unit-level cases had missed entirely. Synthetic random-case testing is good at catching broad regressions in aggregate; it is not a substitute for exercising the real, specific reference scenarios end to end, and it's worth double-checking that a verification script is actually pointed at the code being verified before trusting a "PASS."

## v0.14.13: root-caused the v0.14.12 regression, then re-shipped the optimization with a safety net

### Root cause

Isolated primary 1's own solve -- identical target, identical single obstacle, no sibling primaries yet -- and compared it with and without the clocking-refinement addition:

| | Coarse grid only | With clocking refinement |
|---|---|---|
| Clocking used | 90.0° (a coarse grid point) | 77.6° |
| Min clearance margin | **0.000062"** | **0.000567"** |

The "working" v0.14.11 solution for this primary was already sitting at 0.00006 inches of clearance -- a hair's breadth from touching the keep-out. Adding clocking refinement did exactly what it was designed to do: found a genuinely *better* candidate, with nearly 10x more margin, for that primary considered on its own. That's correct behavior, not a bug.

But this solver works by solving primaries **sequentially**: each accepted primary becomes an obstacle for the next (handoff section 23.3 already documents this -- "an early primary can occupy the only clean corridor for a later one"). Primary 1's new, differently-shaped geometry, despite being strictly safer for itself, occupies the shared 3D volume differently than the old paper-thin-margin version did, and in this specific near-zero-slack scenario that was enough to block every other primary across all four tried solve orders. No bug anywhere in the chain -- every individual candidate along the way was fully valid -- just an emergent property of greedy sequential solving interacting with a scenario that had almost no packing slack to begin with.

### Re-shipped, with a fallback

Given the root cause, the fix is not to abandon the optimization but to add a safety net for exactly the failure mode found: `solve_primary_route()` gained a `dense_search` parameter that restores the full v0.14.11 clocking grid and skips clocking refinement (i.e. the exact old, slower, more conservative search). `_solve_header_primary_set()` (the `Generate All Primaries` group-transaction path) now tries the fast search first across every phase/order combination, and **only if that finds no complete solution at all** retries the entire phase/order search once more with `dense_search=True` before giving up. This pays the old, slower cost exactly in the situation where doing so is worth it: when the faster search already failed outright.

Verified against the same 4-primary reference scenario that exposed the original regression: it now succeeds again, and the internal call counts prove the fallback engaged exactly as designed -- `phase_attempt` showed 4 calls and `order_attempt` showed 16 (3 fast phases × 4 orders, all failing, followed by 1 dense phase × 4 orders, succeeding on the first). Also re-verified against the 6-primary/NORMAL pathological scenario, which succeeded directly through the fast path in 1 phase attempt with no fallback needed, and with a real reduction in collision-check volume (206,617 calls vs. 255,403 before) at comparable total wall-clock time to the pre-optimization baseline. `Generate Active Primary` (single-primary solving) doesn't go through this group-transaction path and isn't affected -- it doesn't have the sequential-blocking exposure this fallback protects against, since there's no later primary for it to block.

## v0.14.14: the same fast/dense pattern extended to keep-out collision resolution

The clocking grid wasn't the only place a "faster but different" search result could turn a marginal Header solve infeasible. `collision_bend_samples` -- the resolution of the candidate's own capsule-chain proxy used during collision testing -- was reduced from the final built Route's resolution back in v0.14.7, decoupling search-time cost from visual smoothness. That reduction is *safe* in the sense that a coarser proxy's sagitta-based radius inflation always over-covers the true swept tube geometry, so it can never let a real collision through. But that same inflation also makes the search's own accept/reject test more conservative than the exact postbuild verification gate: a candidate that clears a keep-out by a hair at full resolution can get rejected here purely from a coarser chord's larger margin, not from an actual collision. In an already-tightly-packed Header, an over-conservative rejection of an otherwise-valid marginal candidate is exactly the same category of problem as the clocking case above -- it can make a later primary's own solve infeasible even though nothing was actually wrong with the rejected candidate.

Given `dense_search` already exists as the recovery mechanism for exactly this class of issue, `collision_bend_samples` now follows it too: the fast default keeps the v0.14.7 coarser resolution, and `dense_search=True` restores the full pre-v0.14.7 density (`max(route_bend_resolution, {'FAST':8,'NORMAL':12,'HIGH':18})`) -- the least conservative, most accurate-to-the-true-geometry proxy chain, applied only on the same fallback retry that already exists for clocking.

Both dimensions of `dense_search` (clocking grid + bend-sample density) are controlled by the single existing flag, so no new API surface or user-facing setting was added -- `_solve_header_primary_set`'s existing fast-then-dense retry now restores both at once on the fallback pass. Re-verified against the same 4-primary and 6-primary reference scenarios: identical results (same order, phase, lengths), with the fallback pass's higher per-call collision-check cost (0.360ms vs. 0.207ms per `primary_collision_check` call, 0.046ms vs. 0.033ms per `candidate_capsule_chain` call) confirming the denser resolution is genuinely engaged during the dense retry, not just the clocking change alone.

## v0.14.15: the per-primary a/b/offset grid, the biggest remaining cost driver

The last major search-density lever: the a/b/offset coarse grid in `solve_primary_route`. Candidate count scales as roughly n³ (a × b × derived offset steps), making this the single biggest cost driver of the three `dense_search` dimensions -- bigger than clocking (which only multiplies the grid, doesn't appear cubed in it) and bigger than bend-sample resolution (which affects per-candidate cost, not candidate count).

Unlike clocking, this dimension didn't need new refinement code: `solve_primary_route`'s local hill-climb around the coarse winner already refines a, b, and offset (it predates all of this work). So reducing the coarse grid here leans on an existing, already-verified mechanism rather than adding a new one -- but it carries the identical category of risk already established for clocking and bend-sample density: a coarser grid can accept or reject a different marginal candidate than the dense grid would, which in a near-zero-slack Header can make a later primary infeasible. Given `dense_search` already exists, this gets the same treatment: fast default (`{'FAST':4,'NORMAL':6,'HIGH':9}`, down from `{'FAST':6,'NORMAL':9,'HIGH':13}`), full original density restored on the fallback retry, no new flag needed.

Verified against both reference scenarios:
- The 4-primary near-zero-slack scenario still needs (and gets) the dense fallback, exactly as before -- and reproduces the *identical* result (same order, same phase, same lengths) as every prior version, confirming `dense_search=True` faithfully replicates the pre-v0.14.13 baseline behavior.
- The 6-primary/NORMAL scenario now succeeds **directly through the fast path** (`phase_attempt`: 1 call, no fallback needed) with `primary_collision_check` dropping from 206,617 calls (v0.14.14) to 113,768 -- a real, direct win with no fallback overhead, taking total solve time from ~99-107s down to 82.6s. It found a different (P3→P4→P5→P6→P1→P2 instead of P6→P5→P4→P3→P2→P1) but equally valid, fully collision-free order -- the multi-order search doing exactly what it's designed to do.

## v0.14.16: the dense-mesh benchmark exposed a budget-starvation bug in the fallback itself

Running the v0.14.15 build against the same realistically complex keep-out mesh from the v0.14.10 benchmark (3072 triangles, not the trivial 12-triangle test cube) surfaced a genuine regression in the fallback machinery: **the scenario failed outright** ("no complete solution found") with the default 60s search budget, where every prior version succeeded with room to spare.

Confirmed by re-running with a 180s budget -- it succeeded, taking 155.3s, via the same fast-fails-then-dense-succeeds pattern already established (`phase_attempt`: 4 calls). The cause: `_solve_header_primary_set` shared *one* deadline between the fast attempt and the dense fallback. On a cheap keep-out mesh that's harmless (the fast attempt fails quickly, leaving the dense fallback most of the budget). On an expensive one, the fast attempt's own doomed exhaustive search can burn through the *entire* shared budget by itself, leaving the dense fallback -- the one actually capable of finding the answer -- with zero time. A scenario that used to succeed comfortably within budget (back when there was only ever one, "dense", search) started reporting a false failure purely because of how the fallback's time got split.

Fixed by giving the fast attempt and the dense fallback **each their own full budget** rather than splitting one shared window: the dense fallback gets a fresh `Max Search Time` allowance starting from when it begins, regardless of how long the fast attempt took failing first. The honest cost: a Header with genuinely no valid solution can now take up to ~2x the configured budget to finally report that, since both searches independently exhaust their own allowance before giving up. Documented explicitly in the property description and the UI panel rather than left as a surprise.

Re-verified: the dense-mesh scenario now succeeds with the *default* 60s budget (122.5s total, fast attempt spending ~60s failing, dense fallback finding the answer within its own fresh 60s) -- same correct result as always (order P1→P2→P3→P4, phase 45°). Both standard reference scenarios (4-primary near-zero-slack, 6-primary/NORMAL) still behave identically to v0.14.15. The artificially-tight-budget test (RUN 4) now correctly takes about twice as long to report "no solution found" (42.3s vs. the previous 25.6s) -- the expected, now-documented consequence of applying the budget twice, not a new bug.

**The recurring lesson across this whole line of work, once more**: this was caught by actually running the realistic benchmark that had been sitting unused since v0.14.10, not by re-testing the trivial cube that every other change in this session had already been verified against. A scenario "already verified" against one test object is not verified against a materially different one -- especially when the metric that failed (search budget, not correctness) was never exercised by the cheap case in the first place.

## v0.14.17: "Complete to Collector" — hand-place the start of a primary, let the solver finish it

New feature, not an optimization: a user can now manually build the first few segments of a Header primary (using the ordinary segment-editing UI, exactly as on any standalone Route) and then click **Complete to Collector** (`exhaust.header_complete_active_primary`) to have the assisted solver route only the *remainder*, from wherever those manual segments end, to the primary's assigned Collector inlet. The hand-placed segments are never touched, re-optimized, or replaced.

This reuses `solve_primary_route` completely unmodified. The solver's bend/clocking math is expressed purely relative to its own local (tangent, up) frame -- never against any fixed global axis -- so re-anchoring the search to start at the end of the manual prefix, instead of at the Route's own local origin, only requires transforming the Collector target and the collision obstacles into a frame anchored there (built from `geometry.route_seam_frames`' exact end-of-prefix pose). The solved segments themselves are plain relative (length, angle, clocking) tuples that compose correctly with the existing prefix the moment `geometry.py`'s turtle-graphics builder walks them -- no new solver code, no new coordinate-conversion step in the segment data itself. An empty segment list (nothing manually placed yet) degrades exactly to "Generate Active Primary": the anchor frame is then simply the Route's own origin/+X/+Z, unchanged.

The manually-placed prefix is also added to the search as a fixed self-collision obstacle, so the solved remainder can't double back into pipe the user already drew, and its own centerline length is subtracted from the Header's equal-length target before the remainder is solved. Getting the self-obstacle right needed one non-obvious fix: the prefix's own last segment always shares its exact endpoint with wherever the solved remainder starts -- that seam is a real, intended connection, not a collision, but a naive capsule-clearance check can't tell the difference from an actual self-intersection there (distance ~0 either way, so *every* candidate was being rejected as "colliding with itself" right at the join). Fixed by trimming the self-obstacle chain near the seam (`_trim_capsule_chain_tail`, `operators.py`) -- with partial clipping of whatever segment straddles the standoff boundary, not a whole-segment drop, since an early version of the fix kept an entire nearby straight run (including the part right at the seam) and reintroduced the same false rejection for a tight manual bend.

Verified with a dedicated standalone-in-Blender test (`test_complete_to_collector.py`, not shipped): a 2-segment manual prefix (straight + 35° mandrel bend) followed by Complete to Collector lands the endpoint on the assigned Collector inlet to within 0.000003" / 0.0000° and a total length within the same normal equal-length-search residual (~0.12") seen on an ordinary "Generate Active Primary" run, with the manual prefix byte-for-byte unchanged before and after; a primary with zero manual segments degrades correctly to a full solve; and a manual prefix aimed away from the Collector with a bend-angle cap too tight to turn back around correctly fails and leaves the prefix untouched. The existing standalone correctness suites (capsule clearance, point-in-mesh, segment/triangle distance) and the full 4-run Blender-headless smoke test both re-verified unaffected.

## v0.14.18: reorganized the Header UI into collapsible panels

Pure UI/UX cleanup, no solver or geometry changes. `ui.py`'s Header inspector used to be one ~290-line method (`draw_header_object`) that rendered every section -- Setup, Primary Tube, Shared Head Flange, Equal-Length Analysis, Collector Target, Assisted Primary Routing (itself containing Guide Splines, Primary Collision Avoidance, Collector Phase Flexibility, Keep-Out Geometry, Search Budget, and Solver Diagnostics, all stacked as plain non-collapsible `box()` groups), and the Primaries list -- fully expanded, all at once, every time any Header was selected. There was no way to collapse a section you weren't using; selecting a Header always meant scrolling past all of it.

Replaced with 14 proper `bpy.types.Panel` sub-panels nested under "Selected Exhaust Object" via `bl_parent_id`, matching how Blender's own native collapsible sections work (a triangle toggle per section, state persisted by Blender itself, no custom code needed for that). Grouping:
- Always-relevant, expanded by default: Setup, Primary Tube, Equal-Length Analysis, Collector Target, Assisted Primary Routing (core solver params + the Generate Active/All buttons + last-solution result stay directly in this one, not buried in a child).
- Occasional-use, collapsed by default: Shared Head Flange (nested directly under "Selected Exhaust Object").
- Advanced/optional, collapsed by default, nested one level deeper under Assisted Primary Routing: Primary Guide Splines, Primary Collision Avoidance, Collector Phase Flexibility, Keep-Out Geometry (Experimental), Search Budget, Solver Diagnostics (Experimental).
- Primaries list stays a sibling of Assisted Primary Routing, expanded by default, at the bottom, matching its prior position.

Each new panel's `poll()` reproduces the exact visibility condition the old code checked inline before drawing that section -- e.g. every Assisted-Routing-family panel requires a Header with a Collector actually assigned (previously the whole block lived inside `if target is not None:`), and Search Budget additionally requires at least one group-search feature (primary-collision avoidance, keep-outs, or phase search) to be enabled, exactly reproducing the old `use_group_search` gate. The Diagnostics panel's profiling on/off checkbox moved into the panel's header row (next to its collapse triangle, via `draw_header()`) rather than the body, so it's visible without expanding the section -- a small but real usability win the old flat-box layout couldn't offer. Two long-dead unused imports (`header_port_index`, `guide_for_route`, pyflakes-confirmed unused before this change too) were removed while touching the same import block.

Verification approach, since `bpy.types.Panel` instances can't be freely constructed outside a real UI region (`--background` mode has no window/UI system at all, confirmed directly: `bpy.types.UILayout` doesn't even expose `row`/`prop`/`box` there) and this environment has no way to open an actual windowed Blender session: (1) `pyflakes` static analysis over the full rewritten `ui.py` -- zero undefined-name findings, which is the class of bug (leftover reference to a variable from the old flat-method scope, e.g. `assist`/`target_box`/`keep`/`phasebox`) this kind of large mechanical refactor is most likely to introduce; (2) a dedicated headless test (`test_ui_registration.py`, not shipped) that registers every class for real, calls every panel's actual `poll()` classmethod against a fully-configured Header/Collector/keep-out scene, and calls every `draw()`/`draw_header()` method body as an unbound function against a fake layout object that accepts and discards every `row`/`column`/`box`/`prop`/`operator`/`label` call -- this exercises every line of real logic (property reads, `header_collector_status`/`primary_collision_pairs`/`keepout_names` calls, string formatting) without needing a real `UILayout`; (3) a second dedicated test (`test_ui_poll_negative.py`, not shipped) checking the negative cases specifically -- all Assisted-Routing-family panels correctly `poll() == False` with no Collector assigned or with a plain (non-Header) Route selected, while Setup/Tube/Flange/Length/Collector-Target/Primaries stay visible regardless, and Search Budget toggles correctly with `solver_avoid_primary_collisions`. All new tests pass; the existing standalone correctness suites and the full 4-run Blender-headless smoke test were re-run and remain unaffected, since no solver, geometry, or operator logic changed -- this is `ui.py` only.

The user should still open the actual Blender GUI to confirm the panels look and collapse as intended -- the verification above proves every panel registers, shows/hides correctly, and draws without raising, but headless testing cannot substitute for eyes on the actual rendered layout.

### Plain-language tooltips on every setting

Same v0.14.18, requested directly ("add tool tips that even an idiot like me can understand"). Before this pass, `properties.py` had 238 property definitions but only 67 carried a `description=` -- everything else fell back to Blender's default tooltip, which is just the property's short label (e.g. hovering "Routing CLR" showed nothing more than "Routing CLR"). Every property actually exposed via `layout.prop()` anywhere in `ui.py` (verified by grepping every `.prop(s, "...")` call, including the dynamically-built `start_`/`end_`/`outlet_` prefixed names used for Route and Collector end-treatment/hardware fields) now has a plain-language description: what the setting does, in everyday words, with jargon either avoided or explained inline the first time it's used (e.g. "CLR" is spelled out as "Centerline Radius: how tight or gentle each bend is" rather than assumed knowledge). 201 of 238 properties now carry one; the remaining 37 are internal flags (`is_header`, `is_route`, ...) and computed read-only display values (`average_length`, `flange_width`, ...) that are only ever shown via `layout.label()`, which has no tooltip in Blender regardless of whether a description is set -- adding one there would be inert, so they were left alone.

Button tooltips (`bl_description` on each Operator) got the same pass: every operator that had no description at all (falling back to its plain button-label text as the tooltip, e.g. hovering "Refresh Header Lengths" showed only "Refresh Header Lengths") now has one explaining what actually happens when clicked, and the more jargon-heavy existing descriptions (e.g. "Choose the radial port offset/order that minimizes current primary endpoint distance") were rewritten in plain terms ("Automatically pick which collector inlet each primary connects to, choosing whichever arrangement keeps pipe ends closest to their inlets").

Verified: registration still succeeds with the longer description strings (no length limit hit), and the full test suite (`test_ui_registration.py`, `test_ui_poll_negative.py`, `test_complete_to_collector.py`, the three standalone correctness suites, and the full 4-run smoke test) all re-run clean, since this only touched `description=`/`bl_description` string literals -- no property types, defaults, update callbacks, or operator logic changed.

### "Complete to Collector" moved to the per-primary Route panel

Also v0.14.18, per direct follow-up request. The button lived in the Header's "Assisted Primary Routing" panel next to "Generate Active/All Primaries" since v0.14.17, but that's the wrong place for it: those two buttons *replace* whatever segments exist, while Complete to Collector's entire point is to *preserve* segments you've already hand-built on the actual selected Route -- grouping it with the replace-everything buttons invited exactly the confusion it's meant to avoid. It now lives on the Route's own panel (shown when a primary is selected directly, not the Header), directly below "Rebuild Exhaust Object" -- right where you're already looking after manually editing that primary's segment list.

First version of this move hid the button entirely until a Collector was assigned to the Header (mirroring the existing "Regenerate This Primary" button just above it in the same panel, which has always worked that way) -- but that made the button invisible with no explanation for anyone who hadn't assigned a Collector yet, which is exactly what happened: it reads as "the button doesn't exist" rather than "the button needs one more step first." Fixed: the button now always shows for any Header primary, right after Rebuild Exhaust Object; it's simply grayed out (`layout.enabled = False`) with an inline "Assign a Collector on the Header to enable this" note when no Collector is assigned yet, so it's always discoverable and the reason it's inactive is stated right there instead of it just being invisible. It's still fully absent (not just disabled) for a plain standalone Route, since the feature is Header-primary-specific by construction.

Verified with a dedicated test (`test_draw_route_button.py`, not shipped) checking `draw_route`'s actual output directly: the button is present-but-disabled with no Collector assigned (and the disabled state itself is checked, not just presence), present-and-enabled positioned after Rebuild once a Collector is assigned, absent entirely for a plain Route, and confirmed gone from the Assisted Primary Routing panel (moved, not duplicated) while Generate Active/All Primaries remain there untouched.

## v0.14 status

The keep-out / collector-phase / guide-spline assisted-routing branch remains experimental/unfinished. v0.14.5 reorganized the physical Header/Collector/Route interfaces; v0.14.6 added solver profiling; v0.14.7 fixed the primary-collision per-call bottleneck that profiling found; v0.14.8 bounds the remaining combinatorial search volume (order comparison count + an overall time budget) so a Header that can't be solved fails fast instead of hanging; v0.14.9 corrects a false non-determinism alarm from v0.14.8's own testing notes (no code change); v0.14.10 fixes the keep-out mesh rebuild redundancy and accelerates the deep-embedding fallback check; v0.14.11 speeds up the exact per-triangle distance primitive via shared precomputation; v0.14.12 adds a verified-safe early-exit contract to that primitive, and separately attempted (then reverted) a reduction to the coarse clocking-search grid after full-pipeline testing caught a regression synthetic testing had missed; v0.14.13 root-causes that regression (an emergent property of sequential greedy multi-primary solving in near-zero-slack scenarios, not a bug) and re-ships the density reduction with a dense-search fallback that recovers exactly the failure mode found; v0.14.14 extends the same fast/dense pattern to keep-out collision-search resolution; v0.14.15 extends it once more to the a/b/offset coarse grid itself -- the single biggest cost driver of the three -- with a measured, verified win on the 6-primary reference scenario (~99-107s down to 82.6s) achieved entirely through the fast path with no fallback needed; v0.14.16 fixes a budget-starvation bug in the fallback itself, found by finally running the realistic dense-mesh benchmark against the accumulated changes -- the fast and dense searches now each get their own full time budget instead of sharing one; v0.14.17 adds "Complete to Collector," letting a user hand-place the start of a primary and have the assisted solver finish only the remainder, reusing the existing solver/geometry machinery unmodified aside from a frame re-anchor and a seam-aware self-collision trim; v0.14.18 is a pure UI/UX pass: replaced the ~290-line always-fully-expanded Header inspector with 14 proper collapsible sub-panels, added plain-language tooltips to every user-facing property and operator button, and moved "Complete to Collector" from the Header's replace-everything Generate buttons to the per-primary Route panel where it belongs alongside Rebuild. All three `dense_search` dimensions (clocking grid, collision bend-sample resolution, a/b/offset grid) are covered by the same single fallback flag and the same fast-then-dense retry in `_solve_header_primary_set`, which now correctly gives each attempt room to actually run. The candidate grid density remains something to tune carefully -- always against real reference scenarios (plural, and varied, not just the cheapest one) -- but the group-solve path now has a safety net covering all three dimensions that can go wrong this way, without that safety net silently starving itself.
