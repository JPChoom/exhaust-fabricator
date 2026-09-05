
# Exhaust Fabricator — Developer Handoff
## Current target: Blender automotive exhaust fabrication add-on
## Current build: v0.14.20 (Primary Count and Port Spacing on a Header now apply automatically -- no more separate "Apply Count"/"Arrange Primary Starts" button click. See section 3, "v0.14.20". Caught and fixed a real double-primary-creation bug in Add Header along the way.)
## Handoff status: v0.14.20 is the latest working branch, but the v0.14.x assisted-routing / keep-out / spline-routing feature set should still be considered experimental and unfinished. The measured primary-collision bottleneck is fixed (5-12x faster), the worst-case search volume is bounded (order-success cap + time budget, each of the fast and dense searches now getting its OWN full budget rather than sharing one), keep-out obstacle rebuilding is no longer redundant per-primary, the exact per-triangle distance math has a correctness-verified early-exit contract, and all three of the coarse clocking-search grid, the collision-search bend-sample density, and the a/b/offset coarse grid itself are reduced with a verified fallback to their old, more conservative values when (and only when) the faster search finds no complete multi-primary solution at all. The failure mode this fallback protects against is understood precisely: solving primaries sequentially means an early primary's own individually-*better* (or merely *less falsely rejected*) candidate can still occupy shared space differently enough to block a later one, purely in near-zero-slack scenarios -- not a coding bug, an emergent property of the solve strategy. The 6-primary reference scenario solves in ~82-83s (down from ~99-107s) via the fast path directly, with no fallback needed; a realistically complex (3072-triangle) keep-out mesh scenario, which briefly regressed to failing outright with the default budget due to the fallback sharing one deadline, now succeeds correctly within that default budget again. v0.14.17 added `exhaust.header_complete_active_primary` ("Complete to Collector"), letting a user manually build the first segments of a primary and have the solver finish only the rest, reusing `solve_primary_route` completely unmodified via a frame re-anchor at the end of the manual prefix, plus a seam-aware self-collision trim (see section 3, "v0.14.17", for the false-self-rejection pitfall this needed to get right). v0.14.18 was a pure UI/UX pass: the Header inspector's single monolithic draw method became 14 proper collapsible `bpy.types.Panel` sub-panels, every user-facing property/operator gained a plain-language tooltip, and "Complete to Collector" moved to the per-primary Route panel under Rebuild Exhaust Object (see section 3, "v0.14.18"). v0.14.19 adds an actual solver capability again: `_candidate()` in `routing_solver.py` now accepts an optional `start_radius` used only for the first bend's fillet trim and arc-length contribution, leaving the other two bends on the standard CLR exactly as before -- letting a Header primary's first bend (nearest the flange) use a tighter CLR for packaging clearance while the rest of the run stays at the standard/sweeping CLR, which is standard real-world header fabrication practice (see section 3, "v0.14.19"). `start_radius` defaults to `None` everywhere and reproduces the exact prior single-radius behavior when omitted or equal to `radius` -- verified, not assumed -- and it is a fixed input, never searched or refined, so it adds zero search cost to the existing solve. v0.14.20 is a small UX fix: Primary Count and Port Spacing now apply automatically via property `update` callbacks instead of requiring a separate button click, and caught a real double-primary-creation bug in `Add Header` along the way (see section 3, "v0.14.20"). No other known correctness issues remain open; the run-to-run non-determinism flagged after v0.14.8 was investigated and traced to comparing two different in-progress code edits, not a real solver property (see the "v0.14.8 addendum").

---

# 1. Project purpose

Exhaust Fabricator is a Blender add-on/extension intended specifically for **automotive exhaust fabrication and header design**, not generic industrial piping.

The primary design goal is to make exhaust components **parametric, fabrication-oriented, editable, and topologically clean** while preserving a practical Blender workflow.

The system is intended to support:

- straight exhaust tubing
- mandrel bends
- pie-cut bends
- reducers / expanders
- radial N→1 collectors
- Y-pipes
- X-pipes
- H-pipes
- route-end treatments
- exhaust flange hardware
- multi-primary headers
- shared header flanges
- header-to-collector integration
- equal-length header analysis
- assisted routing
- primary collision avoidance
- optional keep-out geometry
- optional editable guide splines
- normal downstream exhaust routes attached to collector outlets

The add-on should continue moving toward a practical “design + fabricate” workflow rather than becoming a generic curve-sweep utility.

---

# 2. Highest-level design principles

These principles were established repeatedly during development and should be preserved.

## 2.1 Exhaust-first, not pipe-first

Do not treat every component as “just a tube along a curve.”

The add-on should understand exhaust-specific entities:

- route
- header
- primary
- collector
- Y-pipe
- X-pipe
- H-pipe
- reducer
- flange
- slip socket
- swage
- etc.

The topology, UI, and solver logic should reflect real exhaust fabrication.

## 2.2 Parametric and non-destructive

Whenever practical, the user should be able to alter:

- tube OD
- wall thickness
- CLR
- bend angle
- bend clocking
- straight length
- collector dimensions
- radial phase
- route segment type
- end hardware
- header target length
- routing objective

and have the object rebuild.

## 2.3 Avoid fake intersections

A major repeated lesson in this project:

> Do not make complex exhaust junctions by simply pushing several cylindrical meshes through each other and grouping them.

That approach was explicitly rejected because it produced:

- overlapping mesh
- visible clipping
- disconnected or coincident surfaces
- unrealistic merge regions
- bad topology
- difficult downstream editing

Where components physically merge, the preferred architecture is:

- shared boundary vertices
- single manifold shell where feasible
- real wall thickness
- actual internal passage geometry
- no independent “plug” meshes pretending to be merge spikes

## 2.4 Preserve wall thickness

Exhaust components are intended to be actual hollow tubing where practical.

Wall thickness should remain meaningful through:

- bends
- reducers
- collectors
- Y-pipes
- X-pipes
- H-pipe junctions
- flange bores
- merge webs / collector spikes

## 2.5 Shared topology where parts physically join

Whenever geometry becomes one physical welded/formed component, strongly prefer one connected mesh with shared vertices rather than multiple intersecting submeshes.

This became especially important for radial collectors.

## 2.6 Viewport guides should not render

Segment seams, weld seams, collector target guides, guide splines, etc. should not become accidental render geometry.

Where possible they should be:

- viewport-only overlays, or
- explicitly non-rendering helper curves

Do not pollute final renders, exports, or the Outliner unnecessarily.

## 2.7 Keep the main UI uncluttered

Several redundant creation buttons were removed during development.

Examples:

- collector count buttons were replaced by one `Add Collector` button because N can be edited afterward
- pie-cut bend became a Bend Type inside a Route instead of a separate top-level component

Prefer composition inside the correct parent concept instead of endless top-level buttons.

---

# 3. Version status and validation history

This section is extremely important. Do not assume every version in the history was successful.

## v0.1.0 — validated
Core exhaust route, reducer/expander, radial N→1 collector framework.

User tested and reported it worked as planned.

## v0.2.0 — validated
Collector smooth transition introduced.

Main change:
- no abrupt visual jump from primary OD to collector outlet OD
- smooth multi-lobe to round transition concept

User tested and reported it worked as planned.

## v0.2.1 — validated
Viewport-only route segment seam guides added.

User tested and reported it worked as planned.

## v0.2.2 — failed conceptually
Attempted hover-based segment highlighting.

User reported nothing happened.

Do not reintroduce this tooltip/hover-dependent implementation.

## v0.2.3 — validated
Simplified seam interaction:

- default seam color = medium/dark gray
- selected segment’s starting seam = blue/cyan
- no hover dependency

User tested and reported it worked as planned.

## v0.3.0 — failed
Attempted separate Merge Spike / Knife-Star mesh inserts.

User reported a mess of overlapping geometry.

Do not reintroduce this architecture.

## v0.3.1 — failed
Attempted internal-wall deformation but primaries still clipped through each other.

Still unacceptable.

## v0.3.2 — major improvement
Fixed collector primary overlap by trimming primaries at first tangency and beginning common collector shell there.

User reported it was much better but:
- central hole remained where merge spike should be
- collector body resolution was visibly lower than primary resolution

## v0.3.3 — improved but not final
Filled center hole and improved common collector body resolution.

User then identified:
- merge spike outer shape was convex instead of concave
- spike needed real wall thickness
- primary / collector / spike vertices should line up as one mesh

## v0.3.4 — validated and important
This is the correct radial collector topology foundation.

Key success:
- one shared-vertex manifold collector mesh
- finite-thickness merge web/spike
- concave external merge shape
- correct separation between outer-wall tangency and inner-bore tangency
- no separate spike object
- no overlapping primaries

User said: “Perfect!”

Treat this collector topology as a validated architectural baseline.

## v0.4.0 — validated
Dedicated Y-pipe introduced.

User tested Y-pipe and reported it worked as planned.

## v0.5.0
X-pipe introduced.

## v0.5.1 — validated
Expanded Y-pipe topology families added.

User tested and reported it worked as planned.

Validated Y topology modes:
- Swept Y
- Classic / Straight-Leg Y
- Parallel Merge
- Tangent / Side-Entry Merge
- Formed / Organic Y
- Custom

## v0.6.0 — H-pipe had a junction bug
Lower H-junction showed bad pinwheel/star stitching caused by loop-phase mismatch.

## v0.6.1 — validated
H-pipe saddle stitching corrected.

User tested and reported it was fixed.

## v0.7.0 — pie-cut bend worked
Standalone pie-cut bend functionality worked.

## v0.7.1 — validated
Pie-cut bend integrated into Route Bend Type.

User tested and reported it worked as planned.

This is the preferred architecture:
- Bend Type = Mandrel / Pie-Cut

Do not restore standalone Pie-Cut as the primary UX.

## v0.8.0 — validated
Route end treatments added.

User tested and reported it worked as planned.

Validated end treatments:
- Plain
- Expanded / Swaged
- Reduced / Necked
- Slip Socket

## v0.9.0 / v0.9.1 — flat flange implementations broken
Only `None` and `V-Band` worked correctly.

Do not copy the old generic round plate + holes architecture.

## v0.9.2 — validated
2-bolt and 3-bolt automotive-style flat flanges rebuilt from actual exhaust-flange outlines.

User tested and reported it worked as planned.

Validated route-end connection hardware:
- None
- V-Band
- Round Weld Flange
- 2-Bolt Flat Flange
- 3-Bolt Flat Flange

## v0.10.0 — broken registration
Header feature initially broke add-on registration and all parametric panels.

## v0.10.1 / v0.10.2 — still broken
Panel `draw_header` signature conflict and module caching issue.

## v0.10.3 — registration fixed
New internal module namespace and stale-module purge strategy solved the install issue.

User confirmed it worked.

Important lesson:
- Blender can retain old nested Python modules in `sys.modules`
- new builds may need unique internal namespace or explicit purge logic during experimental registration changes

## v0.10.4 — validated
Header shared generic head flange added.

User tested and reported it worked as planned.

Validated header-side architecture:
- Header primary Start End is not user-configurable
- Header primary Start Connection is not user-configurable
- all primaries share one generic header flange
- primary start-side settings are suppressed/ignored internally

## v0.11.0 — validated
Header-to-Collector assignment and mapping.

User tested and reported it worked as planned.

Validated features:
- assign selected collector
- auto-map ports
- radial mapping
- position error
- angular error
- target guides
- align collector to active primary

## v0.12.0 — validated
Assisted equal-length primary routing.

User tested and reported it worked as planned.

## v0.12.1 — validated
When equal-length target is disabled, `Shortest Possible Route` added.

User tested and reported it worked as planned.

## v0.13.0 — collision avoidance unreliable
Primaries still overlapped.

## v0.13.1 — improved but still imperfect
Conservative bend collision envelope and post-build check added.

Still not always globally collision-free.

## v0.13.2 — validated
Full-set transactional collision-aware header solve.

User tested and said “perfect.”

This is the validated collision-aware routing baseline.

Key behavior:
- solve Header as a complete set
- multiple solve orders
- complete final verification
- all-or-nothing commit
- restore previous state if no fully collision-free solution
- no partially solved overlapping header

## v0.14.0 — experimental / unfinished
Keep-out obstacle routing added.

User said it seemed to be working but was too slow to confidently validate.

Treat this branch as unfinished.

## v0.14.1 — experimental / unfinished
Collector radial phase search added to help solver flexibility.

User said it seemed to be working but was still very slow.

Treat as unfinished.

## v0.14.2 — experimental / unfinished
Per-primary editable non-rendering Bézier guide splines added.

## v0.14.3 — experimental / unfinished
`Splines to Current` added.

Purpose:
- capture current auto-generated primary centerlines into editable guide splines

## v0.14.4 — experimental / unfinished
Generate one primary at a time from the active spline.

Purpose:
- user edits one guide
- solves only that primary
- other primaries remain fixed obstacles
- collector phase remains global and is not adjusted during one-primary solve

## v0.14.5 — current build
Header primary finish-side hardware was removed conceptually and moved to collector outlet.

Current architecture:
- header flange owns upstream interface
- collector inlet owns primary downstream interface
- collector outlet owns downstream end treatment / connection hardware
- normal Exhaust Route can be snapped to collector outlet
- collector can create a new route from its outlet

The v0.14.x routing branch should still be considered experimental because keep-out solving and guide-based solving have not yet been fully performance-validated.

## v0.14.6 — solver profiling instrumentation added (dev tooling, no behavior change)

Added per-stage wall-clock profiling to the assisted-routing solver, per the Step 2 recommendation in section 51.

New module `profiling.py`: a near-zero-overhead stage timer (`stage()` context manager / `mark()`+`record_since()` for multi-exit loops) activated only while `Generate Active Primary` / `Generate All Primaries` runs.

Instrumented stages:
- `candidate_generation` — cost of building one exact-pose filleted-polyline candidate (routing_solver.py)
- `primary_collision_check` / `keepout_collision_check` — split by obstacle kind, inside the same candidate-clearance test
- `candidate_search_coarse` / `candidate_search_refine` — the two search passes in `solve_primary_route`
- `obstacle_build_primary` / `obstacle_build_keepout` — per-primary obstacle assembly cost
- `postbuild_primary_clearance` / `postbuild_keepout_clearance` — final safety-gate verification cost
- `guide_search` / `phase_mapping_heuristic` / `phase_rank` — guide-spline and collector-phase-search overhead
- `primary_solve` — whole `_solve_one_header_primary` call
- `order_attempt` / `fullset_gate_check` — per solve-order attempt in the transactional multi-order search
- `phase_attempt` — per collector-radial-phase candidate in `_solve_header_primary_set`

Results surface two ways: printed to Blender's system console, and shown under Header > Assisted Routing > "Solver Diagnostics — Experimental" (toggle: `solver_profiling_enabled`, default on).

Important: several of these stages nest inside others (e.g. `phase_attempt` contains N `order_attempt`s, each containing per-primary `primary_solve` calls, each containing many `candidate_generation` calls). The report sums wall time per named stage without subtracting nested overlap, so it is a call-tree-style breakdown, not a strict non-overlapping partition — percentages of listed stages need not sum to 100%. This is documented in the profiling module docstring and in the panel.

This is measurement only — no solver behavior, search space, or output geometry changed. It does not resolve the performance concerns in sections 25/26/27/34/47/48; it exists to make the next optimization step evidence-based instead of guesswork.

## v0.14.7 — primary-collision broad phase added (real fix, guided by v0.14.6's profiling data)

Running the new v0.14.6 profiler against a real 4-primary Header (collisions + keep-outs + collector radial-phase search all enabled) overturned the assumption in the original section 51 Step 3: the dominant cost was **not** keep-out mesh collision, it was `primary_collision_check` (sibling-primary capsule-vs-capsule testing) at 82.7% of total solve time — roughly 30x more expensive per call than the keep-out path, because the keep-out path already had a spatial-hash broad phase (section 25) and the primary-vs-primary path had none.

Fix, in `collision.py`/`routing_solver.py`/`operators.py`, search-time only (final built geometry and postbuild safety gates unchanged):
- Added a bounding-box broad phase to `capsule_chain_clearance()`, both whole-chain and per-segment-pair, before the exact segment/segment distance math. Proven safe: a pair is only skipped once its own inflated boxes are proven disjoint, which is only possible when that pair's true clearance is positive, so a real overlap can never be pruned away (full argument in the function's docstring). A fixed sibling-primary obstacle's bounds are now built once (`capsule_chain_bounds()`) and reused across every candidate checked against it in one primary solve, instead of rebuilt per call.
- Decoupled the search's collision-check sample count from the final Route's visual bend resolution (`solver_bend_resolution`) — previously the search silently got more expensive whenever a user increased visual smoothness, even though the sagitta-based radius inflation keeps a coarse proxy chain conservative at any sample count. Collision sampling is now capped independently, at whichever is smaller.

Verified with a 606-case standalone correctness test comparing against a brute-force reference (random + boundary-straddling chain pairs, including genuine collisions) — every real overlap matched exactly; only the reported margin on already-safe pairs can be a conservative overestimate, never wrong-signed.

Measured on the same 4-primary scenario that surfaced the bottleneck:
- Generate All Primaries: 114.0s → 22.8s (5.0x)
- Generate Active Primary (guide-spline solve): 18.7s → 1.5s (12.4x)
- `primary_collision_check` per-call cost: ~21x faster

Both before/after runs produced identical solver output (same order, same phase, same length error) — pure performance change, not a behavior change.

Still unoptimized: keep-out mesh collision itself, candidate generation volume, and the combinatorial size of the phase-search × order-search × candidate-grid space (section 48). A 6-primary Header at NORMAL quality with everything enabled was observed pegging a CPU core for 20+ minutes without finishing during this work and was not pursued further — that combinatorial blowup, not any single per-call cost, is likely the next thing worth addressing.

## v0.14.8 — combinatorial search volume bounded (order-success cap + time budget)

Fixing the per-call collision cost in v0.14.7 didn't fix the 20+ minute hang on the same 6-primary/NORMAL scenario, because the remaining cost is the *volume* of work: phase samples × solve orders × primaries × the per-primary candidate grid (section 48's multiplication, largely unaffected by a per-call speedup). Two changes in `operators.py`/`properties.py`/`ui.py`:

- **`solver_order_success_cap`** (new Header property, default 3): `Generate All Primaries` tries up to 10 different primary solve orders at NORMAL quality and keeps the best-scoring complete one (this breadth is intentional -- see section 23.3, an early primary can hog the only clean corridor for a later one). It previously always tried every generated order even after several had already succeeded, when additional successes beyond a few can only move a low-priority tie-break. It now stops once this many successful orders are found, mirroring the success-cap pattern collector-phase search already used (section 26).
- **`solver_max_solve_seconds`** (new Header property, default 60s): a wall-clock safety ceiling on the combined order/phase search, checked *only between whole attempts, never mid-attempt*. This was a deliberate design correction made during this work: an earlier version also checked between primaries within one order, and testing showed that killed an order that was about to succeed, turning a real (if slow) solution into a false "no solution found" -- worse than the hang it was meant to prevent. Checking between attempts only means total time is bounded by roughly the budget plus one attempt's own worst-case duration, not tightly bounded to the budget itself; pick it alongside Search Quality (which controls a single attempt's duration) rather than expecting the budget alone to cap latency precisely. When the budget runs out, the search stops and reports the same "no solution found, previous routes restored" outcome the all-or-nothing transactional design (section 40) already guarantees for any failed search -- never a partial or colliding commit.

Both surface under Header > Assisted Routing > **Search Budget**, shown whenever collision avoidance, keep-outs, or phase search are enabled.

Measured on the same 20+-minutes-unfinished 6-primary/NORMAL scenario: now finds a complete collision-free solution in 65-106 seconds (some run-to-run variance in which specific order succeeds first and total time -- see caveat below). With an artificially tight budget it correctly reports no-solution-found and restores previous routes rather than running away.

Caveat noticed during testing at the time, later investigated and retracted -- see "v0.14.8 addendum" below: repeated runs of the *identical* scenario appeared to occasionally succeed via a different solve order (forward vs. reverse) and take different total time.

Still unoptimized: keep-out mesh collision's own per-call cost, and the per-primary candidate grid density (a/b/offset/clocking sample counts) — reducing either would need to weigh search thoroughness against speed rather than being a pure win like the last two fixes, so it wasn't attempted here.

## v0.14.8 addendum — the "run-to-run non-determinism" above was a false alarm from comparing different in-progress edits, not a property of the shipped code

Investigated with four independent tests against the actual v0.14.8 code as committed:

1. **In-process repeatability.** The exact 6-primary/NORMAL/collisions+keepouts+phase-search scenario, solved 5 times back-to-back on one unchanging Header object (no rebuild) and 5 more times with the scene fully rebuilt from scratch each time, all within one Blender process. All 10 runs: identical winning order, identical collector phase, identical primary lengths to 4 decimal places, near-identical elapsed time.
2. **Algorithm-only isolation.** `solve_primary_route()` has no `bpy` dependency; loaded standalone (outside Blender entirely, via a minimal `mathutils` stand-in) and called dozens of times with byte-identical fixed inputs, both within one Python process and across several freshly-launched separate `python` processes with confirmed *different* `PYTHONHASHSEED` values each time (printed and verified to differ). Every result was bit-identical regardless.
3. **Separate `blender.exe` process launches**, the actual condition that produced the original observation: the identical single-shot scenario run via genuinely independent process launches (not a loop inside one process) -- 4 launches at FAST quality, 3 more at NORMAL quality, 3 more at NORMAL quality with profiling enabled (matching the original observation's configuration exactly). All 10 launches: identical result, and critically, a SHA-256 fingerprint of the depsgraph-evaluated keep-out mesh's vertex coordinates was also bit-identical across every launch, ruling out Blender-internal mesh evaluation as a variable.

Every test that could be constructed against the actual shipped v0.14.8 code came back deterministic. Reviewing what had actually changed between the two original observations (informally called "test A" and "test C" during that session) showed they were not two runs of the same program: `operators.py` was being actively edited between them (a mid-order per-primary deadline check was added, found to cause a worse failure mode -- see the "v0.14.8" entry above -- and then reverted; the `solver_max_solve_seconds` default and its minimum clamp were also changed in between). The two runs that appeared to disagree were comparing two different in-progress versions of the solve logic, not the same logic run twice. Once the code stopped changing between runs, the disagreement stopped reproducing.

Net finding: no non-determinism bug exists in the current solver. The `min_primary_clearance` broad-phase overestimate noted in `collision.py` (v0.14.7) does not appear to be a contributing factor here -- it affects the *precision* of a reported margin, not the sign of any collision decision, and the fingerprint/algorithm tests above rule out the geometry pipeline as a source entirely. This is a documentation correction, not a code change; v0.14.9 reflects it, but ships identical solver behavior to v0.14.8.

## v0.14.10 — keep-out collision cost, properly this time (with a mesh dense enough to actually show it)

Every prior profiling run (v0.14.6-9) used a bare primitive cube (8 vertices, 12 triangles) as the test keep-out object -- cheap enough that keep-out collision never looked expensive regardless of implementation. Re-profiled against a 3072-triangle mesh (a subdivided cube, standing in for the "engine block / frame rail / transmission" complexity section 25 actually describes): **keep-out collision jumped to 57% of total solve time**. The original section 51 hypothesis (that keep-out collision would be the dominant cost) was directionally right all along -- v0.14.7 corrected it for the trivial-cube case specifically, not in general.

Two changes in `collision.py`/`operators.py`/`routing_solver.py`:

1. **Keep-out obstacle built once per Generate call, not once per primary.** `_collision_obstacles_for_primary()` was rebuilding each keep-out's evaluated mesh, triangulation, and spatial-hash grid from scratch on every call -- once per primary, per order attempt, per phase candidate (hundreds of times per Generate All), even though none of that depends on which primary is being solved. It's now built once, in world space (`_build_world_keepout_obstacles`), threaded down through `_solve_header_primary_set` -> `_solve_header_primary_set_fixed_phase` -> `_solve_one_header_primary`; each candidate's own small capsule chain is transformed into world space at collision-check time instead (cheaper than transforming the whole mesh per primary). Measured: `obstacle_build_keepout` went from 282ms/14 calls to 20ms/1 call on the dense mesh -- confirmed to scale with primary-solve call count, so larger Headers with complex keep-outs benefit proportionally more than this specific 4-primary test shows.

2. **`_point_inside_triangle_mesh`'s "deeply embedded in solid" fallback given a spatial-hash broad phase**, plus its escape ray shortened from `bbox_diagonal*2.5 + 1.0m` (the flat 1-meter term dominates for small mechanical parts, and was directly inflating the new grid-walk's own cost) down to `bbox_diagonal*1.5`. Verified against 400 brute-force-compared test cases across 8 random mesh configurations (interior, exterior, and near-surface boundary points): 0 mismatches. This change is correct, but `cProfile` showed it was **not actually the bottleneck** on the dense-mesh benchmark -- the existing broad phase in the main surface-distance path already prunes to about 1 candidate triangle per capsule segment, and the real remaining cost is that one exact `segment_triangle_distance` computation itself (a Möller-Trumbore intersection test + two closest-point-on-triangle solves + three edge-distance checks, six sub-computations per call). Speeding that up further means reworking a shared geometric primitive other collision paths also rely on for correctness -- a materially higher-risk change than anything done so far -- so it is flagged as the next concrete target rather than attempted here.

Both changes were verified end-to-end against the existing 4-primary and 6-primary scenarios: identical solver output before and after (same order, phase, lengths), on top of their own standalone correctness tests (the point-inside test suite, plus a re-run of the capsule_chain_clearance suite from v0.14.7 since the same file was touched).

## v0.14.11 — segment_triangle_distance sped up via shared precomputation

v0.14.10's `cProfile` pass pointed squarely at `segment_triangle_distance` itself as the remaining bottleneck (not a missing broad phase -- the existing one already narrows each capsule segment to ~1 candidate triangle). It runs six sub-computations per call (Möller-Trumbore plane intersection, two closest-point-on-triangle solves for the segment endpoints, three edge-vs-segment distance checks), and every one of the six was independently re-deriving the SAME two quantities: the segment's own direction vector and the triangle's two edge vectors from vertex A.

Fix, in `collision.py`: compute the segment direction and triangle edges once at the top of `segment_triangle_distance`, and thread them into new precomputed variants of the three sub-routines (`_closest_point_triangle_precomp`, `_segment_triangle_intersection_t_precomp`, `_segment_segment_distance_precomp`) instead of each recomputing from scratch. This is a pure algebraic refactor -- same six checks, same branches, same math, just not repeated six times over.

Verified against the original (unshared) implementation across 3600 cases: bulk random plus deliberately adversarial configurations (piercing segments, zero-length segments, degenerate near-collinear triangles, segments parallel to an edge, points exactly on a vertex/edge) -- every case matched to floating-point tolerance.

Measured on the same 3072-triangle dense-mesh benchmark from v0.14.10: `keepout_collision_check` 2.29ms -> 1.85ms per call (~19%), total solve time 51.4s -> 45.4s (~12%). More modest than the broad-phase/caching wins in v0.14.7/v0.14.10 -- this squeezes an already-fairly-efficient primitive rather than eliminating a missing optimization -- but real, safe, and essentially free. Full pipeline re-verified against the standard 4-primary and 6-primary scenarios: identical solver output before and after.

This closes out the concrete, profiling-identified optimization targets from sections 34 and 48 that were tractable without either (a) reducing search thoroughness (candidate grid density) or (b) a deeper algorithmic rewrite of the collision math with correspondingly higher correctness risk. Both remain open, lower-priority next steps if solve time is still a concern on real-world Header/keep-out geometry.

## v0.14.12 — one of those two next steps shipped, the other was tried and reverted

Attempted both remaining items from the v0.14.11 entry above in one pass.

**Shipped**: gave `segment_triangle_distance` (in `collision.py`) a proper early-exit contract -- `early_exit_below`, so once any of its six sub-checks proves the true distance is at most that value, the rest are skipped, matching the "sign matters, not exact severity once collision is proven" pattern already used for `capsule_chain_clearance` since v0.14.7. The first version of this had a real bug, caught by accident while investigating the regression below rather than by its own dedicated test: it used the collision-radius requirement itself as the early-exit threshold, but the actual outer decision requires `distance - requirement < -1e-10` (strictly more negative). Passing the loose threshold could let an intermediate witness sitting in `[-1e-10, 0)` on one triangle short-circuit the search before a *later* triangle's unambiguous, clearly-negative true collision was ever found -- silently downgrading a real collision to "not rejected." Fixed by passing `required - 1e-10` instead, which provably preserves the exact accept/reject decision the full six-check computation would have made. Verified via 2000 additional threshold-contract cases plus the full-pipeline scenarios.

**Reverted**: halving the coarse clocking-search grid in `solve_primary_route` (it multiplies the entire a/b/offset grid, making it the single largest cost lever) and compensating by adding clocking as a fourth locally-refined dimension in the existing hill-climb pass, the same way a/b/offset are already refined. This tested extremely well synthetically -- 90+ random collision-avoidance scenarios across FAST/NORMAL/HIGH quality, zero success-rate regressions, 1.2-1.9x speedups, length-error deltas from negligible (HIGH) to modest (FAST, up to 21mm on an 0.85m target) -- and then broke the standard 4-primary reference scenario on the very first full-pipeline run after shipping, with primary 1 suddenly reporting "no collision-free route found" where it had solved reliably through every prior version. Confirmed genuinely caused by this change (reproduced against the untouched pre-change baseline side by side), but the exact failure mechanism inside the refinement/scoring interaction was not pinned down with full confidence in the time available, and shipping an unexplained failure mode is a worse trade than not shipping the speed gain. Reverted in full -- `solve_primary_route`'s clocking search is byte-identical to v0.14.11 again.

Worth recording as a process lesson, not just a code note: this regression was only caught because the full-Blender-pipeline verification step happened to run at all. The test script's import path had silently gone stale across three prior version-folder copies earlier in this session and was pointing at the *previous* version's code the whole time -- meaning every "verification" run before the path was noticed and fixed had been testing old code, including runs that reported clean passes. The very first run against the actually-current code caught what 90+ synthetic unit-level cases had missed entirely. Neither aggregate synthetic pass rates nor a green test run should be trusted without first confirming the harness is actually exercising the code under test.

## v0.14.13 — the v0.14.12 regression, root-caused and re-shipped with a fallback

Isolated primary 1's own solve (identical target, identical single keep-out obstacle, no siblings accepted yet) and compared it with and without the reverted clocking-refinement addition:

| | Coarse grid only (v0.14.11) | With clocking refinement |
|---|---|---|
| Clocking used | 90.0° (a coarse grid point) | 77.6° |
| Min clearance margin | 0.000062" | 0.000567" |

The v0.14.11 "working" solution for this primary was already sitting at 0.00006" of clearance -- essentially touching the keep-out. Clocking refinement did exactly what it was built to do: found a genuinely better candidate for that ONE primary, with ~10x more margin. Not a bug. But this solver solves primaries sequentially -- each accepted one becomes an obstacle for the next (section 23.3's own "an early primary can occupy the only clean corridor for a later one") -- and primary 1's new, differently-shaped-but-individually-safer geometry occupied the shared volume differently enough to block every other primary across all four tried orders. No incorrect math anywhere in the chain; an emergent consequence of greedy sequential solving meeting a scenario with essentially zero packing slack to begin with.

Given that root cause, `solve_primary_route()` gained a `dense_search` parameter restoring the exact v0.14.11 clocking grid with no refinement -- the old, slower, proven-conservative search. `_solve_header_primary_set()` now tries the fast (reduced-grid, clocking-refined) search across every phase/order combination first, and only retries the entire phase/order search once more with `dense_search=True` if that found no complete solution at all, before giving up. This pays the old cost exactly when it's worth paying: when the fast search already failed outright.

Verified against the same 4-primary reference scenario that exposed the regression: succeeds again, with internal call counts proving the fallback engaged exactly as designed (`phase_attempt`: 4 calls, `order_attempt`: 16 -- 3 fast phases × 4 orders all failing, then 1 dense phase × 4 orders succeeding on the first). Also re-verified against the 6-primary/NORMAL pathological scenario, which succeeded directly through the fast path (1 phase attempt, no fallback needed) with a real reduction in collision-check volume (206,617 vs. 255,403 calls) at comparable total time to the pre-optimization baseline. `Generate Active Primary` (single-primary solving) doesn't go through the group-transaction path this fallback lives in and isn't affected -- it has no later primary to block, so it isn't exposed to this failure mode in the first place.

## v0.14.14 — the fast/dense pattern extended to keep-out collision-search resolution

The clocking grid was not the only search-time approximation carrying this risk. `collision_bend_samples` (the candidate's own capsule-chain proxy resolution used during collision testing, decoupled from the final built Route's resolution back in v0.14.7) is safe in the sense that a coarser proxy's sagitta-based inflation always over-covers the true swept geometry -- it can never let a real collision through -- but that same inflation makes the search's own accept/reject test more conservative than the exact postbuild gate. A candidate clearing a keep-out by a hair at full resolution can get rejected purely from a coarser chord's larger margin, not an actual collision -- the same category of problem as the clocking regression (an over-conservative rejection of an otherwise-valid marginal candidate blocking a later primary's feasibility in a tightly-packed Header).

Given `dense_search` already exists as the recovery mechanism, `collision_bend_samples` now follows the same split: fast default keeps the v0.14.7 coarser resolution; `dense_search=True` restores the full pre-v0.14.7 density (`max(route_bend_resolution, {'FAST':8,'NORMAL':12,'HIGH':18})`). Both dimensions of `dense_search` (clocking grid and bend-sample density) are driven by the single existing flag -- no new API surface or user setting. Re-verified against the same 4-primary and 6-primary scenarios: identical results, with the fallback pass's higher per-call cost (`primary_collision_check`: 0.360ms vs. 0.207ms; `candidate_capsule_chain`: 0.046ms vs. 0.033ms) confirming the denser resolution is genuinely active during the dense retry.

## v0.14.15 — the a/b/offset coarse grid, the biggest of the three dense_search dimensions

The last major lever: `solve_primary_route`'s a/b/offset coarse grid. Candidate count scales roughly as n^3 (a x b x derived offset steps), making this the single biggest cost driver of the three `dense_search` dimensions -- bigger than clocking (only multiplies the grid, doesn't appear cubed in it) and bigger than bend-sample resolution (affects per-candidate cost, not candidate count).

Unlike clocking, no new refinement code was needed: the local hill-climb around the coarse winner already refines a, b, and offset (predates this whole line of work). Reducing the coarse grid here leans on an existing, already-verified mechanism -- but carries the identical risk class already established for the other two dimensions: a coarser grid can accept/reject a different marginal candidate than the dense grid would, which in a near-zero-slack Header can make a later primary infeasible. Same treatment: fast default (`{'FAST':4,'NORMAL':6,'HIGH':9}`, down from `{'FAST':6,'NORMAL':9,'HIGH':13}`), full original density on the fallback retry, no new flag.

Verified against both reference scenarios: the 4-primary near-zero-slack scenario still needs the dense fallback exactly as before, and reproduces the identical result (same order, phase, lengths) as every prior version -- confirming `dense_search=True` faithfully replicates the pre-v0.14.13 baseline. The 6-primary/NORMAL scenario now succeeds directly through the fast path (`phase_attempt`: 1 call, no fallback needed), `primary_collision_check` dropping from 206,617 calls (v0.14.14) to 113,768 -- a real win with zero fallback overhead, total time ~99-107s -> 82.6s. It found a different but equally valid, fully collision-free order (P3-P4-P5-P6-P1-P2 instead of P6-P5-P4-P3-P2-P1) -- the multi-order search working exactly as designed.

All three `dense_search` dimensions (clocking grid, collision bend-sample resolution, a/b/offset grid) are now covered by the same single flag and the same fast-then-dense retry already in `_solve_header_primary_set` -- no further wiring needed for future dimensions of this kind, should any turn up.

## v0.14.16 — the dense-mesh benchmark exposed a budget-starvation bug in the fallback itself

Finally re-ran the v0.14.15 build against the realistic 3072-triangle keep-out mesh from the v0.14.10 benchmark (not the trivial 12-triangle test cube every other change in this line of work had been re-verified against). Result: the scenario **failed outright** with the default 60s budget, where every prior version succeeded comfortably.

Root cause: `_solve_header_primary_set` shared ONE deadline between the fast attempt and the dense fallback. Harmless on a cheap mesh (fast fails quickly, dense gets most of the budget). On an expensive mesh, the fast attempt's own doomed exhaustive search can burn through the ENTIRE shared budget by itself, leaving the dense fallback -- the one actually capable of finding the answer -- with zero time left. Confirmed by re-running with a 180s budget: succeeded in 155.3s via the expected fast-fails-then-dense-succeeds pattern.

Fix: the fast attempt and the dense fallback now each get their OWN full budget, not a shared window -- the dense fallback's deadline starts fresh from when it begins, regardless of how long the fast attempt took failing first. Honest cost, now documented in the property description and UI: a genuinely infeasible Header can take up to ~2x the configured budget to finally report failure, since both searches independently exhaust their own allowance.

Re-verified: dense-mesh scenario now succeeds with the DEFAULT 60s budget (122.5s total: ~60s fast-attempt failure + dense fallback succeeding within its own fresh 60s), same correct result as always. Both standard reference scenarios unaffected. The artificially-tight-budget failure-reporting test now correctly takes about 2x as long (42.3s vs. 25.6s) -- the expected, documented consequence, not a new bug.

Recurring lesson, stated once more because it recurred: this was caught only because the realistic benchmark -- built in v0.14.10, then left unused while five subsequent versions were verified solely against the trivial test cube -- finally got re-run against the accumulated changes. A scenario verified against one test object is not verified against a materially different one, especially when the metric that broke (search budget exhaustion) is one the cheap case structurally cannot exercise.

## v0.14.17 — "Complete to Collector": hand-place the start of a primary, let the solver finish it

New user-facing capability, not an optimization. Previously, generating a Header primary always meant the solver owned the ENTIRE segment list -- "Generate Active Primary" clears `route.exhaust_route.segments` and rebuilds it from scratch every time, with no way to keep any hand-placed geometry. This adds `exhaust.header_complete_active_primary` ("Complete to Collector"): whatever segments already exist on the active primary's Route when it's clicked are kept exactly as-is, and the solver only searches for the segments needed to reach the assigned Collector inlet from wherever that manual prefix ends.

**Why this needed almost no new solver code.** `solve_primary_route`'s bend/clocking math (`_candidate`, `_turn_from_to` in `routing_solver.py`) is expressed purely relative to its own local (tangent, up) frame, never against any fixed global axis -- the same property the module's own docstring already called out ("Mirrors geometry._segment_bend_basis / route_points update semantics"). That means re-anchoring the search to start at the end of a manual prefix, instead of at the Route's own local origin, only requires transforming the Collector target point/tangent and the collision obstacles into a frame anchored at that pose -- built once via `geometry.route_seam_frames(route.exhaust_route, include_end=True)[-1]`, which already gives the exact position/tangent/up at any segment boundary. The solved candidate's own output (`candidate_segments`: plain relative length/angle/clocking tuples) needs no coordinate conversion at all -- it composes correctly with the existing manual prefix the instant `geometry.py`'s turtle-graphics builder walks the concatenated list, because clocking angles are defined relative to whatever (tangent, up) the walk has already reached, not to any absolute frame. A Route with zero existing segments degrades exactly to "Generate Active Primary" (the anchor frame is then the Route's own origin/+X/+Z) -- no special-casing needed.

Implementation (`operators.py`): a new `frame_local` 4x4 matrix is built from the end-of-prefix pose, with its second (Y) axis deliberately computed as `n_end.cross(t_end)` rather than reusing `route_seam_frames`'s own returned binormal -- that helper's `b` is only guaranteed to be SOME vector perpendicular to (t, n) for display framing, not necessarily the one that keeps the embedding a proper (non-mirrored) rotation; using the wrong-handed one would have silently flipped every bend's effective chirality once composed back into Route-local space. `frame_matrix_world = route.matrix_world @ frame_local` is then passed through unchanged wherever `_solve_one_header_primary` would have used `route.matrix_world` -- `_collision_obstacles_for_primary` gained an optional `local_matrix_world` parameter (default `route.matrix_world`, so the existing call site is untouched) so sibling-primary and keep-out obstacles get expressed relative to the new anchor instead of the Route origin. New `_append_solver_segments` (vs. the existing `_apply_solver_segments`) appends rather than clears.

**The false-self-collision pitfall.** The manually-placed prefix is also added to the search as a fixed obstacle (a capsule chain from `route_world_capsule_chain`), so the solved remainder can't double back into pipe the user already drew. The first attempt at this rejected EVERY candidate outright: the prefix's own last segment always shares its exact endpoint with wherever the solved remainder starts -- that seam is the intended connection, not a collision, but a plain capsule-clearance check can't distinguish "touching by construction at the join" from "genuinely overlapping" (both read as ~0 clearance). Root-caused with a series of standalone diagnostic scripts (`diag_complete.py`, `diag_complete3.py`, not shipped) that isolated the obstacle list and printed the exact candidate/obstacle geometry in solver-local coordinates. Fixed with `_trim_capsule_chain_tail`: walk the prefix's capsule chain backward from the seam, dropping (or partially clipping) whatever falls within a standoff distance (`2 x max(CLR, min straight)`) of the join.

That fix needed a second pass: the first version dropped whole capsule sub-segments once the cumulative distance-from-seam exceeded the standoff, but kept a straddling segment IN FULL rather than clipping it -- for a tight manual bend (2" CLR on a 1.75" OD tube), that left an entire 5" straight run's near end still sitting well within the standoff of the seam, close enough that its 0.875" tube radius plus the new route's own radius exceeded the actual center-to-center gap (~1.2"), producing a genuine but false self-rejection (computed clearance: -0.55"). Fixed by clipping the ONE segment that straddles the standoff boundary at its exact crossing point (via `Vector.lerp`) instead of keeping or dropping it whole, so only the portion of the prefix genuinely beyond the standoff remains as an obstacle.

**Verification.** A dedicated standalone-in-Blender test (`test_complete_to_collector.py`, not shipped) covers: (1) a 2-segment manual prefix (5" straight + 35° mandrel bend, 20° clocking) followed by Complete to Collector, with the two widely-spaced primaries and primary-collision avoidance both enabled -- endpoint position matches the assigned Collector inlet to 0.000003", tangent to 0.0000°, total length within 0.124" of the equal-length target (the same order of residual an ordinary "Generate Active Primary" run leaves, since this is a discrete grid search, not an exact solve), and the manual prefix's 2 segments are byte-for-byte unchanged before and after; (2) a primary with zero manual segments correctly degrades to a full solve; (3) a manual prefix deliberately aimed away from the Collector combined with a bend-angle cap too tight to turn back around correctly fails (`{'CANCELLED'}`) and leaves the prefix untouched (a `None` candidate is detected before any segments are appended, so no snapshot/restore is even needed for this failure path). All three existing standalone correctness suites (capsule clearance: 606 cases, point-in-mesh: 400 cases, segment/triangle distance: 3600+2000 cases) and the full 4-run Blender-headless smoke test were re-run and remain unaffected -- this feature is purely additive (a new operator, a new solve function, one new optional parameter on `_collision_obstacles_for_primary` with a backward-compatible default); no existing code path's behavior changed.

UI: a new "Complete to Collector" button (`ui.py`, next to "Generate Active Primary" / "Generate All Primaries" in the assisted-routing panel) with inline label text explaining the intended workflow (manually build the start, then complete the rest).

## v0.14.18 — reorganized the Header UI into collapsible panels

Pure UI/UX cleanup requested directly ("let's look at the UI and clean it up, organize it"); no solver, geometry, or operator behavior changed.

**The problem.** `ui.py`'s `EXHAUST_PT_Selected.draw_header_object` had grown into one ~290-line method that rendered the ENTIRE Header inspector -- Setup, Primary Tube, Shared Head Flange, Equal-Length Analysis, Collector Target (mapping/tolerances/status), Assisted Primary Routing (core solver params, plus nested `box()` groups for Guide Splines, Primary Collision Avoidance, Collector Phase Flexibility, Keep-Out Geometry, Search Budget, and Solver Diagnostics), and the Primaries list -- fully expanded, every time, with no way to collapse anything. `layout.box()` only draws a visual border; it has no expand/collapse state at all. Selecting any Header meant scrolling past all ~15 sections regardless of which one you actually needed.

**The fix.** Split the monolithic method into 14 real `bpy.types.Panel` subclasses, each with `bl_parent_id = "EXHAUST_PT_selected"` (or nested one level deeper under the new `EXHAUST_PT_header_assisted_routing`), giving every section Blender's native collapsible-panel behavior (a triangle toggle, state persisted by Blender itself across the session -- no custom property or extra code needed for that part). Grouping, chosen by how often each section is actually touched:
- Expanded by default, direct children of "Selected Exhaust Object": `EXHAUST_PT_HeaderSetup`, `EXHAUST_PT_HeaderTube`, `EXHAUST_PT_HeaderLength`, `EXHAUST_PT_HeaderCollectorTarget`, `EXHAUST_PT_HeaderAssistedRouting` (core solver params + the Generate Active/All buttons + the last-solution result box all stay directly in this one, not pushed into a child -- these are the actions someone opening this panel is there to use), `EXHAUST_PT_HeaderPrimaries` (the per-primary length list, at the bottom, same position as before).
- Collapsed by default: `EXHAUST_PT_HeaderFlange` (direct child; set-once-then-rarely-touched), and nested one level under Assisted Routing: `EXHAUST_PT_HeaderGuides`, `EXHAUST_PT_HeaderPrimaryCollision`, `EXHAUST_PT_HeaderPhase`, `EXHAUST_PT_HeaderKeepouts`, `EXHAUST_PT_HeaderSearchBudget`, `EXHAUST_PT_HeaderDiagnostics` -- all advanced/optional/experimental, matching how rarely each is toggled relative to the core Generate workflow.

Each panel's `poll()` reproduces exactly the visibility condition the old inline code checked before drawing that section, so nothing became MORE visible than before by accident: every Assisted-Routing-family panel requires `header_target_collector(obj) is not None` (previously the entire block lived inside `if target is not None:`), and `EXHAUST_PT_HeaderSearchBudget` additionally requires the same `use_group_search` condition as before (primary-collision avoidance, or keep-outs with at least one assigned, or phase search enabled). Two small helper functions, `_header_settings(context)` and `_header_with_target(context)`, centralize those checks so all 14 `poll()` methods stay one-liners instead of re-deriving the header/target lookup independently.

One small usability improvement fell out of the mechanical split rather than being a separate change: `EXHAUST_PT_HeaderDiagnostics` moved its profiling on/off checkbox into the panel's header row via `draw_header()` (Blender's per-panel header-row hook, drawn next to the collapse triangle even while the section is collapsed) instead of inside the body -- so profiling can now be toggled without expanding the (collapsed-by-default) Diagnostics section at all. Also removed two long-dead unused imports (`header_port_index`, `guide_for_route`) noticed via `pyflakes` while touching the same import block; confirmed both were already unused before this change, not orphaned by it.

**Verification, and its limits.** This environment cannot open a real windowed Blender session, and `bpy.types.Panel` subclasses cannot be freely instantiated outside one -- confirmed directly: in `--background` mode, `bpy.types.UILayout` doesn't even expose `row`/`column`/`prop`/`box`/`operator` via `dir()`, since the interface/RNA-function layer isn't registered without a window manager. Given that hard constraint, verification used three layers instead: (1) `pyflakes` static analysis of the fully rewritten `ui.py` -- zero undefined-name findings, which is precisely the bug class a mechanical split-one-method-into-many is most likely to introduce (a leftover reference to a variable that only existed in the old method's local scope, e.g. `assist`, `target_box`, `keep`, `phasebox`, `guidebox`, `diag`); (2) a new headless test (`test_ui_registration.py`, not shipped) that registers every class for real via `ef.register()`, builds a fully-configured Header/Collector/keep-out scene, calls every panel's actual `poll()` classmethod, and calls every `draw()`/`draw_header()` method body as an unbound function (`cls.draw(fake_self, context)`) against a fake layout object that accepts and silently discards every `row`/`column`/`box`/`prop`/`operator`/`label`/`separator` call -- this exercises every line of real logic (property reads, `header_collector_status`/`primary_collision_pairs`/`keepout_names` calls, f-string formatting against real Header/Route data) without needing an actual `UILayout`, and would raise on any `AttributeError`/`NameError`/`TypeError` a broken draw body could produce; (3) a second new test (`test_ui_poll_negative.py`, not shipped) specifically covering the negative cases -- every Assisted-Routing-family panel correctly returns `poll() == False` with no Collector assigned, and every Header-family panel (all 14) correctly returns `poll() == False` when a plain non-Header Route is selected instead, while `EXHAUST_PT_HeaderSetup`/`Tube`/`Flange`/`Length`/`CollectorTarget`/`Primaries` stay `True` regardless of Collector assignment, and `EXHAUST_PT_HeaderSearchBudget` flips from `False` to `True` exactly when `solver_avoid_primary_collisions` is turned on. All new tests pass. The existing standalone correctness suites (capsule clearance: 606 cases, point-in-mesh: 400 cases, segment/triangle distance: 3600+2000 cases) and the full 4-run Blender-headless smoke test were re-run against the reorganized build and remain unaffected -- expected, since no solver/geometry/operator code changed, only confirmed rather than assumed.

What this verification does NOT establish, and the user should still check directly in the Blender GUI: that the panels visually nest, collapse, and read the way intended -- headless testing proves every panel registers, shows/hides under the right conditions, and draws without raising, but it cannot substitute for actually looking at the rendered sidebar.

### Plain-language tooltips (same v0.14.18)

Direct follow-up request: "add tool tips that even an idiot like me can understand". Audited `properties.py`: 238 property definitions, only 67 had a `description=` set. Blender falls back to the property's short label as the tooltip when none is given, so e.g. hovering "Routing CLR" or "Merge Lobe Strength" showed nothing beyond the label itself -- no help for anyone unfamiliar with the term.

Cross-referenced every `layout.prop(s, "...")` call in `ui.py` (via `grep -oP '\.prop\(\s*\w+,\s*"\K[a-z_0-9]+'`) to find which properties are actually shown to a user, including the dynamically-built `f"{prefix}_..."` names used throughout Route's start/end treatment and connection-hardware fields and Collector's `outlet_` fields -- nearly every property in the file turned out to be reachable this way. Rewrote/added a `description=` on all of them, in plain English: what the setting physically does, phrased for someone with no CAD/Blender background, defining jargon inline the first time a term appears in a group (e.g. "CLR" is introduced as "Centerline Radius: how tight or gentle each bend is" rather than assumed knowledge; "V-Band Flange" as "a round clamp-style flange ... quick to assemble/disassemble" rather than left unexplained). Coverage went from 67/238 to 201/238; the remaining 37 are internal boolean flags (`is_header`, `is_route`, ...) never exposed via `prop()`, and computed `SKIP_SAVE` display values (`average_length`, `flange_width`, `computed_*`, ...) shown only via `layout.label()`, which never renders a tooltip regardless of whether a description is set -- confirmed by checking each remaining name is not among the `.prop()`-exposed set before leaving it alone, rather than assuming.

Did the same pass over every `bl_description` in `operators.py`: added one to every operator that had none (falling back to its bare button-label text as the tooltip, e.g. "Refresh Header Lengths" hovering to just "Refresh Header Lengths"), and rewrote the more jargon-dense existing ones in plain terms (e.g. "Choose the radial port offset/order that minimizes current primary endpoint distance" became "Automatically pick which collector inlet each primary connects to, choosing whichever arrangement keeps pipe ends closest to their inlets"; "Create a generalized radially symmetric N-to-1 collector" became "Create a collector that merges any number of primary pipes into one outlet, evenly spaced around a center").

Verified: `py_compile` and `pyflakes` clean on both files (pyflakes does flag every Blender-style `name: SomeProperty(...)` class-level annotation as a spurious "syntax error in forward annotation" -- confirmed this is a pre-existing false positive by running it against the untouched v0.14.17 `properties.py` too, not something introduced by adding more/longer description strings). Registration succeeds in Blender with the longer strings (no tooltip length limit hit), confirmed via `test_ui_registration.py`/`test_ui_poll_negative.py`/`test_complete_to_collector.py` all re-run clean, plus the three standalone correctness suites and the full 4-run smoke test -- unaffected, since only string literals changed, no types/defaults/callbacks/operator logic.

### "Complete to Collector" relocated to the per-primary Route panel (same v0.14.18)

Third follow-up request, same session: "move the button 'Complete to Collector' button to the selected primary section under the 'Rebuild Exhaust Object' button." The button had been placed in the Header's `EXHAUST_PT_HeaderAssistedRouting` panel next to "Generate Active/All Primaries" when it shipped in v0.14.17 -- but those two buttons *replace* whatever's on the primary, while Complete to Collector's whole purpose is to *preserve* what's already there and only fill in the rest. Grouping it with the replace-everything actions worked against the feature's own point.

Moved to `EXHAUST_PT_Selected.draw_route` (the panel shown when a primary Route is selected directly, not the Header), immediately after the existing `exhaust.rebuild` ("Rebuild Exhaust Object") button call -- the same place a user is already looking right after hand-editing that primary's segment list.

**Follow-up bug: the button was invisible with no explanation.** The first version gated the button's very presence on `header is not None and header_target_collector(header) is not None`, mirroring how "Regenerate This Primary" (just above it in the same panel) has always been gated. The user reinstalled the new build and reported "do not see the 'Complete to Collector' button anywhere" -- traced to exactly this: with no Collector yet assigned to the Header, the button doesn't exist in the panel at all, which reads as "this feature doesn't work" rather than "assign a Collector first." Fixed by decoupling *visibility* from *availability*: the button now always draws for any Header primary (still never for a plain standalone Route, since the feature is Header-primary-specific by construction), wrapped in its own `layout.row()` with `.enabled` set to `header_target_collector(header) is not None` -- Blender's standard way to show a greyed-out, unclickable button rather than hide it -- plus an inline `layout.label(text="Assign a Collector on the Header to enable this", icon='INFO')` when it's disabled, so the reason is stated rather than implied. This is a UX principle worth remembering for this codebase: a control that's sometimes valid and sometimes not should normally be shown-but-disabled-with-a-reason, not hidden outright, unless its very existence would be actively confusing in the invalid state (e.g. a per-segment PIE_CUT-only field when the segment is a STRAIGHT) -- "Regenerate This Primary" likely has the same latent discoverability problem and could get the same treatment if it comes up again.

Verified with a dedicated test (`test_draw_route_button.py`, not shipped) that calls `EXHAUST_PT_Selected.draw_route` directly against a recording fake layout (one where nested `row()`/`column()`/`box()` calls all write into the same shared call log as the root and each call also records the enabling layout's `.enabled` state at that point, matching how a real rendered panel's grey-out actually works -- an earlier version of this same test had sub-layouts each keeping their own private log, which silently hid every button drawn inside a `row()`, including Generate Active/All Primaries; fixed before trusting any of its results) and inspects both the sequence and the enabled-state of `operator()` calls made: present-but-disabled with no Collector assigned, present-and-enabled positioned strictly after `exhaust.rebuild` once one is assigned, absent entirely for a plain Route, and confirmed no longer present in `EXHAUST_PT_HeaderAssistedRouting`'s own draw output while `exhaust.header_solve_active_primary`/`exhaust.header_solve_all_primaries` remain there untouched.

## v0.14.19 — mixed CLR: a tighter bend near the flange, standard CLR for the rest

Prompted by a direct fabrication-practice question, not a bug report: is it common in real headers to mix centerline radii on the same primary? Researched and confirmed yes -- 1.0D-1.25D (D = tube OD) tight bends right off the cylinder head flange to clear a steering shaft, frame rail, or motor mount, transitioning to the standard 1.5D+ (sometimes 2.0-3.0D for sweeping/high-flow builds) CLR for the rest of the run, is standard practice in both aftermarket fabrication and OEM cast/hydroformed manifolds. The assisted solver had no way to express this: `solve_primary_route`'s 3-corner filleted-polyline candidate used exactly one CLR for all three bends.

**The change.** `_candidate()` in `routing_solver.py` gained an optional `start_radius` parameter, applied only to the FIRST corner (nearest the search's own start pose, i.e. nearest the flange in a normal full solve) -- the other two corners keep using `radius` (the standard/Routing CLR) exactly as before. Concretely: `radii = (R1, R, R)` where `R1 = start_radius if given else R`, and both the per-corner fillet trim (`q = r * tan(theta/2)`) and the total-length arc contribution (`sum(r * theta for r, theta in zip(radii, thetas))`) now use the corresponding per-corner radius instead of one shared `R`.

Three downstream functions previously read a single `candidate['radius']` for every bend and needed the same per-bend treatment: `candidate_segments()` (assigns each generated BEND segment's `radius` field), `candidate_centerline_points()` (samples the exact centerline for diagnostics/guide probes), and `candidate_capsule_chain()` (builds the collision-search proxy chain, including the sagitta-based radius inflation that keeps it conservative). All three now read a `candidate['radii']` 3-tuple, falling back to `(candidate['radius'],) * 3` if a caller ever constructs a candidate dict without it (defensive, not expected to trigger).

**New Header settings** (`properties.py`): `solver_use_tight_start_clr` (bool, off by default) and `solver_start_clr` (float, DISTANCE, same default as `solver_clr`). UI (`ui.py`): a checkbox next to Routing CLR that reveals a compact inline Start CLR field only when enabled, in `EXHAUST_PT_HeaderAssistedRouting`. Threaded through in `operators.py`: `_solve_one_header_primary` passes `start_radius=hs.solver_start_clr if hs.solver_use_tight_start_clr else None` to `solve_primary_route`.

**Deliberately NOT threaded into "Complete to Collector"** (`_solve_one_header_primary_completion`): that feature's entire premise (v0.14.17) is that the user has already hand-built the tight, packaging-constrained portion of the primary themselves, before clicking Complete to Collector to auto-route the remainder. By the time that solve runs, the tight-clearance zone near the flange is normally already behind the search's own starting pose -- applying Start CLR there would tighten the SOLVER's own first generated bend a second time, in a location that's no longer anywhere near the flange. Left as an explicit code comment at that call site rather than a silent omission, since it's the kind of thing that looks like an oversight if undocumented.

**No new search cost.** `start_radius` is never searched, refined, or treated as a free variable -- it's a fixed input representing a real physical constraint (a specific CLR die/tube available, or a specific clearance a fabricator already knows they need), set once by the user. This keeps it outside the `dense_search` risk category entirely: there's no coarse grid, no fast/dense fallback question, and no interaction with the sequential multi-primary packing sensitivity documented for `dense_search`'s three dimensions above -- it's a pure input parameter, not a search dimension.

**Backward compatibility, verified not assumed.** `start_radius` defaults to `None` at every level (`_candidate()`, `solve_primary_route()`, and both existing `operators.py` call sites), and `None` reproduces the exact prior single-radius formula (confirmed via `test_mixed_clr.py`: `start_radius=None` and no-`start_radius`-argument-at-all produce bit-identical `total_length`/`lengths`; `start_radius` explicitly equal to `radius` also matches exactly). Every existing caller of `solve_primary_route` that doesn't pass the new keyword is completely unaffected.

**Verification** (`test_mixed_clr.py`, not shipped): beyond the backward-compatibility checks above, confirms a genuinely different `start_radius` changes only the first bend's trim/arc-length contribution (the coarse cross-check against an independently-recomputed arc-length formula matches to ~1e-6, the small residual being ordinary floating-point divergence between two different angle-computation code paths -- `_angle_between` directly vs. `_turn_from_to`'s rotation composition -- not a real discrepancy, confirmed by the exact endpoint check below); `candidate_segments()` assigns the tighter radius to exactly the first BEND segment and the standard radius to the rest; the solved centerline still lands exactly on the target position (~1e-6") and tangent (~0.01°) regardless of which radius the first bend uses, since the fillet-trim math guarantees exact endpoint/tangent by construction independent of radius choice; and a full Blender-headless pipeline test confirms `Generate Active Primary` with `Tighter CLR Near Flange` enabled produces a route whose first bend is the Start CLR and remaining bends are the Routing CLR, while disabling the toggle reproduces the prior uniform-CLR output exactly. The existing standalone correctness suites (capsule clearance, point-in-mesh, segment/triangle distance), the UI test suite (registration, poll negatives, Complete-to-Collector, the relocated-button test), and the full 4-run Blender-headless smoke test were all re-run against the new build and remain unaffected.

## v0.14.20 — Primary Count and Port Spacing apply automatically

Small direct UX request: "when changing primary count 'Apply Count' and 'Arrange primary Starts' automatically, same when clicking Port spacing. The buttons themselves don't need to be visible." Previously, typing/dragging a new `desired_primary_count` or `port_spacing` value on a Header did nothing by itself -- you had to separately click "Apply Count" (`exhaust.header_apply_count`) or "Arrange Primary Starts" (`exhaust.header_arrange_starts`) in the Setup panel for the change to actually add/remove primaries or re-space them.

**The fix.** `EXHAUST_OT_HeaderApplyCount.execute()`'s body (the add/remove-primaries-until-count-matches loop, reindex, active-primary clamp, flange rebuild, stats refresh) was factored out into a new shared function `_apply_header_primary_count(header_obj)` in `operators.py`, so it's callable from outside an operator's `execute()`. `properties.py` gained two new `update` callbacks on `EXHAUST_PG_Header`: `_update_header_primary_count` (lazy-imports and calls `_apply_header_primary_count` then `_arrange_header_starts` then rebuilds the flange -- a primary-count change needs re-spacing too, not just the count fixed) and `_update_header_port_spacing` (lazy-imports and calls just `_arrange_header_starts` then rebuilds the flange). Both follow the exact lazy-import-inside-the-callback pattern already established for `_update_header_flange`/`_update_header_tube_size` in the same file, needed because `operators.py` imports from `properties.py` at module level, so the reverse import can only happen lazily, after both modules are fully loaded. `desired_primary_count` and `port_spacing` were wired to these callbacks, and their tooltips updated to say the change applies immediately rather than describing a button to click. The two buttons were removed from `EXHAUST_PT_HeaderSetup`'s draw body; the two operators themselves are untouched and still registered, so anything that wants to re-trigger them directly (search menu, a future keymap, a script) still can.

**A real bug this caught before shipping, not after.** `EXHAUST_OT_AddHeader.execute()` used to set `hs.desired_primary_count = self.primary_count` and then *also* run its own `for i in range(self.primary_count): _create_header_primary(header, i)` loop -- harmless while the property had no update callback, but the moment the callback was wired up, setting the property silently created `self.primary_count` primaries via the callback, and the very next line created `self.primary_count` more, doubling the Header's primary count (requesting 4 produced 8). This wasn't found by manual inspection; a new test (`test_auto_apply_arrange.py`) asserting the primary count immediately after `Add Header` failed first (4 requested, 8 found), which is exactly the point of asserting concrete counts rather than just "the operator returned FINISHED." Fixed by deleting the now-fully-redundant manual creation loop (along with the equally-redundant explicit `_arrange_header_starts`/`rebuild_header_flange_object` calls right after it, since the callback already does both) -- `Add Header` now just sets `desired_primary_count` once and lets its own update callback do the rest, matching how every other caller of primary-count changes now behaves.

**Verification** (`test_auto_apply_arrange.py`, not shipped): raising `desired_primary_count` from 4 to 6 adds exactly 2 primaries and re-arranges all 6 to the expected evenly-spaced positions with no operator call anywhere in the test; lowering it to 3 removes exactly 1; changing `port_spacing` re-arranges all primaries to the newly-spaced expected positions with no operator call; a direct check of `EXHAUST_PT_HeaderSetup.draw()`'s actual output confirms neither `exhaust.header_apply_count` nor `exhaust.header_arrange_starts` is drawn anymore; and both operators are confirmed still independently callable and still correct when invoked directly (registered, not deleted). The existing standalone correctness suites, the mixed-CLR suite, the full UI suite (registration, poll negatives, Complete-to-Collector, the relocated-button test), and the full 4-run Blender-headless smoke test were all re-run and remain unaffected -- the smoke test in particular re-confirms `Add Header` produces exactly the requested primary count end-to-end, not just in the isolated new test.

---

# 4. Current object/component model

The add-on currently revolves around several parametric object types.

## 4.1 Exhaust Route

A Route is the standard continuous exhaust tube path.

A Route contains ordered segments.

Supported segment types:

### Straight
Parameters:
- length

### Bend
Parameters:
- Bend Type
  - Mandrel
  - Pie-Cut
- CLR / Equivalent CLR
- bend angle
- clocking
- bend resolution
- pie section count when Pie-Cut

A Route may mix:

- Straight
- Mandrel Bend
- Straight
- Pie-Cut Bend
- Straight
- Mandrel Bend

within one object.

### Route-level parameters
Typical parameters include:
- outside diameter
- wall thickness
- profile/circumferential resolution
- route segment collection
- seam guide visibility
- seam guide offset
- endpoint treatments
- endpoint connection hardware

Exception:
Header-managed primaries suppress endpoint treatment/hardware as described later.

---

# 5. Route seam system

This is a validated UX feature and should not be regressed.

Each route segment boundary is shown as a viewport-only ring.

Default seam:
- medium/dark gray

Selected segment starting seam:
- blue/cyan

Behavior:
- selecting a segment in the Route segment list highlights that segment’s starting seam
- no hover dependency
- no render geometry
- no helper mesh in Outliner

The original hover attempt failed and should not be revisited using tooltip timing.

---

# 6. Mandrel bends

Mandrel bends use:

- outside diameter
- wall thickness
- CLR
- bend angle
- 3D clocking
- bend resolution

Clocking rotates the next bend plane around the current route tangent.

The Route construction should preserve a local frame:
- position
- forward/tangent
- up
- right

This enables compound 3D header geometry.

Centerline arc length:

L = R * theta

where theta is in radians.

---

# 7. Pie-cut bends

Pie-cut is a Bend Type inside Route.

Do not make it a separate top-level component in the normal UX.

Geometry concept:
- each pie section is a straight cylindrical tube section
- section ends are true miter planes
- shared miter boundaries form elliptical seams
- route remains one connected hollow mesh

Inputs:
- Equivalent CLR
- total bend angle
- clocking
- pie section count

Displayed calculations may include:
- weld seam count
- angle per weld
- section centerline length
- total pie centerline length

Pie centerline approximation used:

L_section = 2 * R * sin(delta_theta / 2)

Total = N * L_section

Weld seam guides:
- viewport-only
- actual miter/ellipse location
- no render

---

# 8. Reducer / Expander

Dedicated parametric component.

Core parameters:
- inlet OD
- outlet OD
- length
- wall thickness
- profile resolution

Designed for exhaust transitions.

---

# 9. Radial N→1 Collector

This is one of the most carefully developed areas.

## 9.1 User-facing architecture

There is one:
- `Add Collector`

Do not restore separate 2:1, 3:1, 4:1, etc. creation buttons.

Primary Count is editable afterward.

Supported range:
- 2–12

## 9.2 Radial symmetry

Collector primary centers are radially symmetric:

phi_i = phi_0 + 2*pi*i/N

Inputs include:
- Primary Count
- Primary OD
- Outlet OD
- Wall Thickness
- Collector Length
- Transition length
- Radial Phase
- radial spread / packing controls
- profile resolution
- transition resolution

## 9.3 Auto packing

Radial packing can derive the center radius from:

R = (D + G) / (2 * sin(pi/N))

where:
- D = primary OD
- G = desired gap
- N = primary count

## 9.4 Correct collector topology

The final validated collector architecture from v0.3.4 is extremely important.

Do not regress to overlapping cylinders.

Correct sequence:

1. separate hollow primaries
2. primary outer walls converge
3. outer walls reach first tangency
4. at this tangency, the common outer shell begins
5. inner bores remain distinct farther downstream
6. inner bores reach tangency later
7. this difference creates real merge-web/spike wall thickness
8. geometry transitions into one round hollow outlet

The collector merge spike is not a separate mesh.

The merge spike/web is the metal remaining between the internal flow passages as they converge.

This gives:
- concave outer valley
- finite thickness
- shared vertices
- one manifold body

## 9.5 Collector body resolution

The common body should retain edge density comparable to individual primaries.

Avoid visibly lower segment count through the collector body.

The collector common body resolution was previously scaled by primary count to prevent obvious faceting.

---

# 10. Collector outlet treatment and hardware — current v0.14.5 architecture

As of v0.14.5, downstream fabrication options belong on the Collector outlet.

Collector Outlet End:
- Plain
- Expanded / Swaged
- Reduced / Necked
- Slip Socket

Collector Outlet Connection:
- None
- V-Band
- Round Weld Flange
- 2-Bolt Flat Flange
- 3-Bolt Flat Flange

This is intentional.

Do not put individual Finish End / Finish Connection hardware on Header primaries.

---

# 11. Y-Pipe

Dedicated 2→1 topology.

Do not treat Y-pipe merely as a radial 2:1 collector special case.

The Y-pipe uses a saddle/merge architecture appropriate to two branches.

Validated topology families from v0.5.1:

## 11.1 Swept Y
Existing smooth baseline.

## 11.2 Classic / Straight-Leg Y
Most of each branch remains straight before a tighter terminal sweep.

## 11.3 Parallel Merge
Parallel inlet tangents with configurable center spacing.

Useful for long-divider / long merge designs.

## 11.4 Tangent / Side-Entry Merge
One dominant/main run.
One side branch enters at angle.
Outlet can be biased toward the main run.

Useful for many real OEM and aftermarket automotive Y-pipes.

## 11.5 Formed / Organic Y
Softer continuous curvature resembling formed commercial exhaust pieces.

## 11.6 Custom
Exposes:
- spacing
- independent A/B angles
- straight-leg fraction
- signed outlet bias

The user explicitly requested topology variety based on real automotive reference examples.

Future Y development should preserve this concept:
branch routing and merge topology are separate concerns.

---

# 12. X-Pipe

Dedicated 2→2 crossover component.

The design separates:
- branch routing
- crossover topology

Modes introduced:
- Classic X
- Swept X
- Parallel Entry
- Custom

Important:
The X should be a true communicating internal crossover, not merely two intersecting cylinders.

Supports:
- symmetric/asymmetric A/B geometry
- inlet spacing
- outlet spacing
- independent approach angles
- crossover length
- crossover opening
- plane rotation
- resolution

The internal passages should join and separate with wall thickness preserved around the crossover saddle.

---

# 13. H-Pipe

Dedicated H-pipe / balance tube component.

Parameters:
- Main A OD
- Main B OD
- wall thickness
- main pipe length
- center spacing
- crossover OD
- crossover longitudinal position
- crossover angle
- blend length
- plane rotation
- main/junction/profile resolution

Important bug history:
v0.6.0 lower saddle stitching had a 180-degree loop-phase mismatch producing pinwheel triangles.

v0.6.1 fixed this by phase-aligning saddle and crossover loops before stitching.

Do not regress this correspondence logic.

The H-pipe should be one manifold mesh rather than three cylinders simply overlapping.

---

# 14. Route end treatments

Validated from v0.8.0.

For normal standalone Routes, each end may use:

- Plain
- Expanded / Swaged
- Reduced / Necked
- Slip Socket (Female)

Typical parameters:
- target OD
- transition length
- straight end length
- socket depth
- diametral clearance

Slip socket logic:

Socket ID = route OD + diametral clearance

Socket OD = Socket ID + 2 * wall thickness

Header-managed primaries are a special case and should not expose these start/finish treatments.

---

# 15. Connection hardware

Validated from v0.9.2.

Connection types:
- None
- V-Band
- Round Weld Flange
- 2-Bolt Flat Flange
- 3-Bolt Flat Flange

## 15.1 V-Band
This worked early and should remain stable.

Typical controls:
- flange OD
- weld neck length
- taper length
- face width

## 15.2 Round Weld Flange
Uses corrected deterministic opening tessellation.

## 15.3 2-Bolt Flat Flange
Must resemble an automotive exhaust flange, not a generic round plate.

Important parameters used:
- A = overall height
- B = overall width
- C = bolt-center spacing
- flange thickness
- bolt hole diameter
- rotation

The perimeter is based on:
- central bore body
- two bolt ears
- automotive elongated outline

## 15.4 3-Bolt Flat Flange
Rounded triangular / three-ear flange.

Parameters:
- A overall height
- B overall width
- thickness
- bolt hole diameter
- rotation

Do not revert to generic circular outer perimeter.

---

# 16. Header system

Header is a coordinator object that manages multiple normal Exhaust Route primary objects.

Supported primary count:
- 2–12

Each primary remains a real Route.

This is important because each primary can still contain:
- straights
- mandrel bends
- pie-cut bends
- independent 3D routing

## 16.1 Header coordinator

Stores/manages:
- primary count
- primary OD
- wall thickness
- initial primary length
- port spacing
- target length
- equal-length tolerance
- target reference mode
- collector assignment
- primary-to-collector mapping
- assisted routing controls
- spline guide controls
- keep-out controls

## 16.2 Parent/child management

A safer architecture was introduced after v0.10.0 registration failures.

Header primaries should be managed through:
- parent/child relationship
- lightweight metadata such as primary index

Avoid fragile nested Blender RNA collection structures that can break add-on registration.

---

# 17. Shared Header flange

Validated in v0.10.4.

Header primary starts do not have individual connection hardware.

Instead all primary starts share one generic header flange.

Current generic flange:
- rounded rectangular plate
- auto-fits around current primary start locations
- one real through-bore per primary

Controls:
- flange thickness
- edge margin
- corner radius
- port bore clearance
- corner segments
- computed overall width/height

Header primary Start End:
- suppressed
- internally forced Plain

Header primary Start Connection:
- suppressed
- internally forced None

This should remain true even if an old .blend contains different hidden values.

Future possible improvements:
- engine-specific head flange profiles
- oval ports
- D-ports
- rectangular ports
- imported profile
- bolt patterns
- non-uniform port spacing

But current generic shared flange is validated.

---

# 18. Header primary finish architecture — current v0.14.5 decision

The user decided Header primaries also do not need individual:

- Finish End
- Finish Connection

Reason:
the collector is the downstream termination hardware.

Therefore Header primary endpoints should be treated as:

- upstream → shared Header flange
- downstream → assigned Collector inlet

No individual primary endpoint fabrication hardware should exist.

Internally force:
- Finish End = Plain
- Finish Connection = None

Normal standalone Routes still retain both endpoint treatment/hardware systems.

---

# 19. Equal-length analysis

Header continuously tracks primary centerline lengths.

Typical display:

P1 31.875"  Δ -0.125"
P2 32.040"  Δ +0.040"
P3 32.000"  Δ  0.000"
P4 31.960"  Δ -0.040"

Statistics:
- average
- shortest
- longest
- spread
- individual deviation
- tolerance status

Length Reference:
- Target Length
- Current Average

Tools:
- Target = Active
- Copy Active Segment Layout to All
- Edit Active Primary

Equal-length should be based on centerline length.

---

# 20. Header → Collector integration

Validated in v0.11.0.

Header may be assigned to a radial Collector.

Each collector inlet is a separate target port.

Features:
- Assign Selected Collector
- Auto Map Ports
- Port Offset
- Reverse Port Order
- target guides
- live position error
- live angular error
- position tolerance
- angle tolerance
- Align Collector to Active Primary

A primary target is:
- collector inlet position
- collector inlet tangent

Port mapping is part of the Header/Collector relationship.

---

# 21. Assisted Primary Routing — validated base

The first successful assisted routing release was v0.12.0.

The solver creates normal editable Route segments, not a permanent arbitrary spline.

Generated routes can include:
- Straight
- Mandrel Bend
- Straight
- Bend
- etc.

The solver should enforce:
- start position
- start tangent
- collector target position
- collector target tangent
- routing CLR
- minimum straight
- maximum bend angle
- dogleg offset
- dogleg clocking

Generated output must remain manually editable.

---

# 22. Routing objectives

## 22.1 Match Equal-Length Target

When enabled:
- satisfy collision constraints
- hit collector endpoint/tangent
- get within equal-length tolerance
- among valid equal-length solutions choose shortest / least-excursive route

The user explicitly requested:

> “Match Equal-Length Target” should also look for the shortest route possible while avoiding intersecting/overlapping with another primary.

Therefore the scoring priority should remain:

1. hard validity
2. collision-free
3. endpoint/tangent constraints
4. length inside tolerance
5. shortest route among qualifying candidates
6. lower excursion / gentler route as tie-breaker

Do not treat “any route near target length” as sufficient.

## 22.2 Shortest Possible Route

When Equal-Length is disabled, user can enable:
- Shortest Possible Route

This should minimize actual centerline length within the routing topology while preserving hard constraints.

If both:
- Equal-Length OFF
- Shortest OFF

then a compact/gentle feasible-route mode may remain.

---

# 23. Primary-to-primary collision routing

This required several iterations.

## 23.1 Failed simple sampled centerline approach

v0.13.0 used approximate sampled centerline capsules and still allowed overlaps.

Problem:
bend chords lie inside the true circular centerline.

## 23.2 Conservative bend envelope

v0.13.1 improved the test using per-segment capsules with bend sagitta compensation.

For a bend chord, expand the collision envelope enough to contain the true circular sweep.

This architecture should remain.

## 23.3 Full-set transactional solver — validated v0.13.2

This is the correct Header collision-routing model.

Do not solve P1→P2→P3→P4 once and assume success.

Generate All should:
- try multiple solve orders
- try multiple clocking phases
- build full candidate Header
- verify all primary pairs
- commit only complete collision-free set

Possible orders:
- forward
- reverse
- cyclic
- outside-in
- center-out

Transactional behavior:
- snapshot entire Header primary set
- if attempt fails, discard attempt
- if all attempts fail, restore exact previous Header
- never leave partially solved overlapping geometry

This was user-tested and explicitly called “perfect.”

Treat v0.13.2 as the validated collision-aware routing core.

---

# 24. Primary clearance

Routing can require a minimum surface-to-surface clearance.

Primary Clearance = 0:
- physical tangency allowed
- overlap forbidden

Positive value:
- require actual gap between outer tube surfaces

Collision check should use tube OD, not only centerlines.

---

# 25. Keep-Out Geometry — experimental / unfinished

Introduced in v0.14.0.

Purpose:
allow user-defined Blender meshes representing:
- engine block
- frame rail
- steering shaft
- transmission
- bellhousing
- oil pan
- subframe
- firewall
- custom clearance volumes

Workflow concept:
- select Header + keep-out meshes
- Header active
- Add Selected Meshes
- set Keep-Out Clearance
- assisted solve avoids them

Collision model concept:
- evaluated mesh
- spatial acceleration
- segment-to-triangle distance
- tube radius + keep-out clearance
- closed solid volume checks

Important:
User said this seemed to work but took a long time and could not be confidently validated.

Therefore:
- DO NOT describe keep-out routing as fully validated
- performance is an open problem

Potential optimization work is needed.

---

# 26. Collector Radial Phase search — experimental / unfinished

Added in v0.14.1.

Purpose:
give solver extra flexibility without moving the collector body.

During Generate All:
- collector position/orientation fixed
- radial inlet pattern may rotate around collector axis
- port mapping may be remapped cyclically

Unique search range is bounded by radial symmetry.

For N inlets:
unique period = 360° / N
meaningful search can be clamped to ± half that pitch

Examples:
- 4:1 → ±45°
- 6:1 → ±30°
- 8:1 → ±22.5°
- 12:1 → ±15°

Search Quality controls phase sample count.

This is global:
- do not automatically change Collector Radial Phase during a one-primary solve because it moves every primary target

This feature is currently experimental because overall v0.14 routing performance is unfinished.

---

# 27. Primary Guide Splines — experimental / unfinished

Added beginning v0.14.2.

Purpose:
give the designer a manual directional hint so solver does not search blindly.

## 27.1 Guide objects

Each Header primary can have a non-rendering editable Blender Bézier curve.

Guide:
- starts near flange port
- ends near mapped collector inlet
- editable in Blender Edit Mode
- user may move handles / subdivide
- non-rendering
- not the final exhaust geometry

The final route is still composed of:
- Straights
- Mandrel Bends
- etc.

## 27.2 Soft preference

The spline is not a hard route.

It should influence:
- preferred dogleg plane
- preferred excursion
- approximate routing corridor
- candidate pre-ranking

Hard constraints still win:
- collisions
- CLR
- bend angle
- endpoint/tangent
- length objective

Guide Influence controls strength.

Guide solve may fall back to full search if no valid guided result exists.

---

# 28. Splines to Current

Added in v0.14.3.

Button:
- `Splines to Current`

Purpose:
convert currently modeled / auto-generated primary centerlines into editable guide splines.

Expected behavior:
- create missing guides
- overwrite guide geometry
- guide follows current primary route closely
- simplify dense bend samples enough to remain editable
- reset accidental object transforms so guide aligns with route

Typical workflow:

1. Auto-generate Header
2. Splines to Current
3. Edit guides manually
4. Solve again

---

# 29. Generate Active from Spline

Added in v0.14.4.

Purpose:
allow one primary at a time to be generated from its guide.

Important workflow:

1. select P1 guide
2. edit spline
3. Generate Active from Spline
4. only P1 is regenerated
5. all other current primaries remain fixed
6. they are treated as collision obstacles
7. repeat for P2, P3, etc.

Collector Radial Phase should NOT be adjusted during single-primary solve.

Generate All remains available as the global alternative.

---

# 30. Collector outlet → normal Route workflow

Current v0.14.5 direction.

The collector is the downstream interface after the Header.

Collector panel should support:
- Add Route from Outlet

This creates a normal Exhaust Route aligned to the collector outlet.

Normal Route should support:
- Snap Start to Selected Collector

Workflow:
1. select collector
2. select route
3. route active
4. Snap Start to Selected Collector

Snap should use actual exposed connector positions, including collector outlet treatment/hardware.

The normal Route then continues downstream with all normal route features.

---

# 31. Connector metadata

Connector metadata should track actual exposed endpoint geometry.

Useful metadata:
- world/local endpoint position
- tangent direction
- OD
- ID
- connection type
- hardware type

When a treatment or flange extends the physical endpoint, the connector location should move to that true exposed mating face.

This is important for snapping.

---

# 32. Blender extension packaging lessons

The project is packaged as a Blender Extension.

Expected root structure includes:
- `blender_manifest.toml`
- `__init__.py`
- Python package/modules

Important registration lessons from v0.10.x:

## 32.1 Avoid reserved Panel callback names
`Panel.draw_header` has a reserved signature:

draw_header(self, context)

Do not create unrelated helpers named `draw_header(self, layout, obj)`.

## 32.2 Registration failures can destroy all parametric UI
A failure during class registration can prevent:
- PropertyGroups
- pointer properties
- panel classes
from being registered, making all objects appear non-parametric.

Use defensive registration and rollback.

## 32.3 Blender may retain modules in memory
Uninstalling an extension may not always purge nested Python modules from `sys.modules` during the same Blender session.

Possible mitigation:
- unique internal module namespace for major hotfix builds
- explicit purge of prior Exhaust Fabricator submodules before import

This solved v0.10.2/v0.10.3 issues.

---

# 33. UI philosophy

The main panel should remain relatively simple.

Current creation hierarchy should approximately be:

- Add Header
- Add Exhaust Route
- Add Reducer / Expander
- Add Collector
- Add Y-Pipe
- Add X-Pipe
- Add H-Pipe

Do NOT add a separate Pie-Cut Bend button.

Do NOT restore N-specific Collector buttons.

Contextual features belong inside their parent object panels.

Examples:
- pie cut belongs in Route Bend
- shared head flange belongs in Header
- collector outlet hardware belongs in Collector
- guide spline features belong in Header routing
- route-to-collector snap belongs in Route / Collector

---

# 34. Performance concerns

The major unresolved issue is v0.14 assisted-routing performance.

The user reported:
- keep-out solve appears to work
- phase search appears to work
- but it takes a long time

Performance should become a major focus before adding substantially more search dimensions.

Potential optimization directions:

## 34.1 Broad-phase acceleration
Use cheap bounds before expensive collision tests.

Examples:
- AABB
- capsule bounds
- BVH
- spatial grid

## 34.2 Cache obstacle acceleration structures
Do not rebuild evaluated-mesh BVHs for every candidate.

Cache per:
- keep-out object
- evaluated depsgraph state
- transform hash

## 34.3 Cache existing primary collision representations
Current primaries are fixed obstacles for many candidate tests.
Precompute their capsule/sagitta envelopes.

## 34.4 Guided search first
Guide spline mode should reduce candidate search dramatically.

Potential strategy:
- derive preferred dogleg plane from guide
- search only near guide first
- widen adaptively if no solution

## 34.5 Hierarchical candidate scoring
Do not run expensive exact obstacle collision tests on obviously bad candidates.

Suggested order:
1. parameter validity
2. endpoint feasibility
3. bend-angle validity
4. cheap distance lower bounds
5. primary broad phase
6. obstacle broad phase
7. exact collision tests
8. objective scoring

## 34.6 Parallelization
Be careful with Blender API thread safety.

Pure mathematical candidate evaluation may be parallelizable if all Blender data is copied into plain Python/numpy-like structures first.

Do not access Blender RNA from worker threads.

## 34.7 Progressive search
Could offer:
- Draft
- Normal
- High

and show candidate count / progress.

However do not introduce UI complexity unless solver latency remains problematic.

---

# 35. Recommended next development step

The most logical next step is NOT another major geometry primitive.

The best next step is to stabilize and optimize the current v0.14 routing branch.

Recommended sequence:

## Step 1 — validate v0.14.5 physical hierarchy
Confirm:
- Header primaries no longer expose Finish End / Finish Connection
- Collector outlet treatment/hardware works
- Add Route from Outlet works
- Snap Start to Selected Collector works
- connector positions reflect hardware

## Step 2 — profile solver performance
Instrument:
- phase candidates
- dogleg candidates
- collision checks
- keep-out triangle tests
- guide-biased candidate count
- time spent per stage

Add optional developer diagnostics only if necessary.

## Step 3 — optimize keep-out collision
This is likely the current dominant cost.

Use cached BVHs and aggressive broad-phase pruning.

## Step 4 — improve spline-guided solve
The spline should do more than influence scoring.

Potential upgraded approach:
- resample guide
- derive several local routing frames
- fit a small number of manufacturable bend/straight corners to the guide
- then optimize those corners rather than sweeping large global dogleg search

This may dramatically reduce solve time.

## Step 5 — preserve full automatic fallback
Guide mode must remain optional.

Default:
- current automatic solver

Optional:
- guide-assisted solver

This was explicitly requested.

---

# 36. Potential future features after routing stabilizes

These are ideas consistent with the project direction.

## 36.1 Engine-specific header flange templates
Examples:
- LS family
- S54
- common small-block patterns

Could support:
- port position
- port shape
- bolt holes
- head-face outline

## 36.2 Custom imported flange profile
User imports:
- SVG
- DXF
- Curve
- Mesh outline

Header ports map into that flange.

## 36.3 Custom port shapes
Current routing assumes round primary tube.

Header flange port could support:
- round
- oval
- D-port
- rectangular
- custom

Transition from head port to round primary could be parametric.

## 36.4 Primary grouping
For 8-cylinder:
- 4:1 per bank
- tri-Y
- 4-2-1
- 8:1
- 180-degree crossover header

This will require routing graph topology rather than only one final collector.

## 36.5 Multi-stage collector systems
Examples:
- 4-2-1
- 6-3-1
- tri-Y
- 8-4-2-1

The generalized graph model should eventually allow nested collectors.

## 36.6 Fabrication reports
Potential output:
- tube OD
- wall
- straight cut lengths
- bend angles
- CLR
- clocking
- pie-cut weld count
- total centerline length
- flange dimensions

Could generate CSV / PDF later.

## 36.7 Tube bend schedule
For each primary:
- segment number
- straight length
- bend angle
- CLR
- rotation/clocking from previous bend
- accumulated centerline length

## 36.8 Weld seam visualization / numbering
Could number:
- pie-cut welds
- route segment seams
- collector interfaces

## 36.9 Real clearance heat map
Color-code routes by minimum clearance to:
- other primary
- keep-out
- chassis

Viewport-only.

## 36.10 Collector phase live preview
Instead of only solver-driven phase search, provide a viewport manipulator or slider preview.

---

# 37. Geometry requirements by component

## Route
- hollow
- continuous
- correct OD/ID
- clean normals
- editable segments

## Mandrel Bend
- real CLR
- true circular centerline
- correct clocking

## Pie-Cut
- true straight tube sections
- true mitered ends
- shared seam vertices

## Reducer
- hollow transition
- no arbitrary overlap

## Collector
- no primary overlap
- trim at outer tangency
- shared outer shell
- inner bores merge later
- finite-thickness merge web
- radial symmetry
- consistent edge density

## Y
- one connected merge
- topology styles must genuinely differ
- no intersecting cylinders

## X
- true internal crossover communication
- finite wall around crossover
- one manifold shell

## H
- real saddle openings
- loop phase alignment at both junctions
- no star/pinwheel stitching

## Flanges
- real through bores
- bolt holes
- automotive perimeter where relevant

---

# 38. Solver hard constraints vs soft objectives

This distinction should remain explicit in code.

## Hard constraints
Must be satisfied or candidate is invalid.

Examples:
- valid CLR
- max bend angle
- minimum straight
- exact/near-exact target position
- exact/near-exact target tangent
- no primary overlap
- keep-out clearance when enabled
- valid geometry
- no degenerate route

## Soft objectives
Used to rank valid candidates.

Examples:
- match guide spline
- shortest route
- equal-length closeness
- minimal excursion
- lower clocking deviation
- fewer/lower-angle bends
- smaller collector radial phase change

Do not allow soft score to override a collision.

---

# 39. Equal-length solver scoring recommendation

For Match Equal-Length Target:

Candidate validity:
- collision-free
- endpoint valid
- tangent valid
- fabrication constraints valid

Then scoring:
1. whether within target tolerance
2. absolute length error
3. total route length
4. spline deviation if guide enabled
5. dogleg excursion
6. bend severity
7. collector phase change

For full Header set:
1. entire set collision-free
2. all primaries inside tolerance if possible
3. minimize maximum deviation
4. minimize total primary length
5. maximize minimum clearance
6. minimize total excursion
7. minimize collector phase change

---

# 40. Full-set transactional behavior

This is mandatory for `Generate All Primaries`.

Pseudo-flow:

1. snapshot all current primary segment data
2. snapshot collector radial phase
3. snapshot port mapping
4. generate candidate set
5. verify every primary
6. verify every primary-primary pair
7. verify keep-outs if enabled
8. verify collector targets
9. only if entire set passes:
   - commit
10. otherwise:
   - restore all snapshots

Never leave a partial solve in the scene.

This behavior was specifically validated in v0.13.2.

---

# 41. One-primary spline solve behavior

For `Generate Active from Spline`:

- solve only associated primary
- other primaries remain unchanged
- other primaries act as collision obstacles
- use current fixed collector radial phase
- use current fixed port mapping
- do not globally remap ports
- do not rotate collector
- if solve fails:
  - restore only active primary
  - do not change others

This is deliberately different from Generate All.

---

# 42. Spline behavior recommendation

Guide spline should be used as a design hint.

Potential future stronger integration:

1. sample spline at normalized arc length
2. compute local tangent
3. identify major curvature changes
4. reduce spline to a handful of routing waypoints
5. fit manufacturable line + arc primitives
6. optimize waypoint positions to:
   - maintain CLR
   - avoid collisions
   - maintain target length

This could eventually replace much of the broad blind dogleg search.

But preserve:
- soft-guidance mode
- automatic fallback

---

# 43. Header flange + collector ownership summary

The current intended physical ownership is:

## Upstream
Cylinder head
→ Shared Header flange
→ Header primaries

## Header termination
Header primaries
→ Collector inlet ports

Header primaries:
- no individual start hardware
- no individual finish hardware

## Downstream
Collector body
→ Collector outlet treatment
→ Collector outlet connection hardware
→ Normal Exhaust Route

This is the current cleanest architecture.

---

# 44. Normal Exhaust Route snapping

Normal Routes should support two mechanisms:

## Existing generic snap
Useful for arbitrary exhaust objects.

## Collector-specific
`Snap Start to Selected Collector`

Should:
- align Route start connector to Collector exposed outlet connector
- match position
- match tangent direction
- account for collector end treatment
- account for collector flange/V-band
- account for route start hardware where applicable

Collector should also support:
`Add Route from Outlet`

This should create a correctly aligned normal Route.

---

# 45. Backward compatibility philosophy

Where old properties must remain for `.blend` compatibility:

- keep the properties registered
- hide/suppress them contextually
- force safe internal behavior for managed objects

Examples:
Header primary old Start/Finish hardware properties may remain, but geometry engine ignores them.

Avoid deleting properties if it would break older files unless a migration system exists.

---

# 46. Testing checklist for every future release

## Installation
- uninstall previous version
- install new ZIP
- no registration exception
- no missing parameter panels

## Existing objects
Select and inspect:
- Route
- Reducer
- Collector
- Y
- X
- H
- Header

All parametric settings should still appear.

## Route
- Straight
- Mandrel Bend
- Pie-Cut Bend
- seam highlighting
- mixed route

## Collector
- 4×2" → 3" is the canonical visual test
- no primary overlap
- concave merge
- wall thickness
- shared mesh
- smooth body resolution
- radial phase

## Y
Test:
- Swept
- Classic
- Parallel
- Tangent
- Formed
- Custom

## X
Test multiple topology modes.

## H
Inspect both saddle junctions for phase stitching.

## End treatments
- Expanded
- Reduced
- Slip Socket

## Flanges
- V-band
- Round
- 2-bolt
- 3-bolt

## Header
- primary count change
- shared flange
- equal-length statistics
- collector mapping
- target guides

## Assisted routing
- equal-length
- shortest
- primary collision
- Generate All transaction
- guide mode
- active guide solve
- keep-out mode
- radial phase mode

## Collector outlet
- outlet treatment
- outlet hardware
- Add Route from Outlet
- Snap Route to Collector

---

# 47. Known experimental areas

These areas should be labeled experimental in code/comments/docs until fully validated:

- keep-out routing
- collector radial-phase optimization inside automatic solve
- spline-guided routing
- spline-based one-primary solve
- performance of full v0.14.x search stack

The geometry primitives themselves are much more mature than the v0.14 solver layer.

---

# 48. Known performance risk

The main performance risk is combinatorial search growth:

phase samples
× port mappings
× solve orders
× dogleg offsets
× dogleg clocking samples
× length candidates
× primary collision tests
× keep-out tests
× number of primaries

This can explode quickly.

Do not simply add more sampling density to improve quality.

Prefer:
- better heuristics
- spline guidance
- spatial pruning
- caching
- adaptive search
- local refinement after coarse solve

---

# 49. Recommended internal architectural separation

Maintain separation among:

## Geometry generation
Responsible for mesh.

Should not know about global search strategy.

## Parametric data
PropertyGroups / object metadata.

## UI
Panels/operators.

## Connector system
Endpoint/port metadata and snapping.

## Header coordination
Parent/child management, equal-length stats, mappings.

## Routing solver
Pure geometry/math candidates where possible.

## Collision system
Primary capsules, bend sagitta envelopes, keep-out BVH.

## Guide spline system
Create/select/update/sample guides.

This separation reduces the chance that adding a solver feature breaks all object property panels as happened in v0.10.0.

---

# 50. Handoff instructions for the next developer / coding agent

1. Start from the latest v0.14.5 package, not an older branch.
2. Preserve every feature validated through v0.13.2.
3. Treat v0.14.x automatic keep-out/phase/spline routing as experimental.
4. Do not rewrite mature geometry unless there is a reproducible defect.
5. Keep Header primaries free of individual endpoint hardware.
6. Keep collector outlet as the downstream header interface.
7. Preserve collector v0.3.4 topology.
8. Preserve H-pipe v0.6.1 loop-phase fix.
9. Preserve flat-flange v0.9.2 shapes.
10. Preserve v0.13.2 all-or-nothing full-set solve.
11. Optimize before expanding the global routing search space further.
12. Prefer guide-spline-driven local search as the path toward faster user-controlled routing.
13. Maintain Blender extension registration safety.
14. Avoid reserved Blender callback names.
15. Preserve unique-module / stale-module mitigation if refactoring registration.
16. Continue syntax/AST checks before packaging.
17. Keep a clear versioned ZIP per test build.
18. User testing in Blender is the final acceptance gate.

---

# 51. Current recommended next task

The immediate next task should be:

## Stabilize v0.14.5 and optimize assisted routing performance

Specifically:

### A. Verify v0.14.5 hierarchy changes
- Header primaries have no Start/Finish endpoint hardware UI.
- Collector outlet owns treatments/hardware.
- Add Route from Outlet works.
- Snap Start to Selected Collector works.

### B. Add solver profiling
Measure:
- candidate generation time
- primary collision time
- keep-out collision time
- phase-search time
- guide search time

### C. Cache keep-out BVHs
Avoid rebuilding obstacle acceleration structures per candidate.

### D. Use guide spline to directly seed manufacturable route corners
Do not only use it as a score.

A good first implementation:
1. sample guide
2. find 2–4 major direction-change points
3. create virtual polyline
4. fillet corners using routing CLR
5. test against hard constraints
6. locally optimize waypoint positions
7. fall back to broader search only if needed

This should reduce solve time far more effectively than increasing blind candidate count.

---

# 52. Project identity summary

Exhaust Fabricator should continue evolving toward:

> A Blender-based parametric automotive exhaust and header fabrication system where tubing, bends, merges, collectors, crossovers, flanges, and header routing behave like real exhaust fabrication components rather than generic overlapping meshes.

The strongest parts of the project are currently:
- parametric Route system
- mandrel/pie-cut bends
- radial collector topology
- Y/X/H junctions
- automotive flange geometry
- Header coordinator
- shared head flange
- equal-length analysis
- Header-to-Collector mapping
- transactional primary collision solver

The weakest / unfinished area is:
- high-complexity assisted routing performance once keep-out geometry, radial-phase search, and guide-spline search are all enabled simultaneously.

That should be the primary engineering focus before adding significantly more automatic-routing dimensions.

---

# 53. Final non-regression rules

Do not regress any of the following:

- gray segment seam guides
- blue/cyan active segment seam
- pie-cut as Route Bend Type
- single Add Collector button
- collector 2–12 editable N
- radially symmetric collector inlets
- v0.3.4 shared-vertex collector topology
- multiple Y topology modes
- X topology modes
- corrected H saddle stitching
- route end treatments
- automotive 2-bolt and 3-bolt flange shapes
- shared Header flange
- Header primaries as normal editable Routes
- Header-to-Collector port mapping
- equal-length stats
- shortest-route mode
- v0.13.2 transactional group collision solve
- optional guide spline mode
- Splines to Current
- one-primary Generate Active from Spline
- collector outlet as downstream header interface
- route snapping to collector outlet

---

End of handoff.
