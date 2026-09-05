"""Lightweight wall-clock profiling for the assisted-routing solver.

The v0.14.x solver stack (keep-out collision, collector radial-phase search,
guide-spline routing) has never been performance-validated -- the handoff
notes repeatedly flag it as slow but unmeasured.  This module gives the
solver a near-zero-overhead way to record where time actually goes, without
threading a profiler object through every function signature.

Usage:
    from .profiling import SolverProfile, active_profile, stage

    profile = SolverProfile()
    with profile.active():
        ... run solver code that calls stage("candidate_generation") etc ...
    report = profile.report_text()

When no profile is active, `stage()` is a near-free no-op (one attribute
lookup and an `if`), so leaving the instrumentation in place costs nothing
during normal interactive use.
"""
import time
from contextlib import contextmanager

_ACTIVE = None


class SolverProfile:
    """Accumulates wall-clock time and call counts per named solver stage."""

    def __init__(self):
        self.totals = {}
        self.counts = {}
        self.start_time = None
        self.end_time = None

    @contextmanager
    def active(self):
        """Install this profile as the process-wide active profiler.

        Blender's Python is single-threaded for add-on execution, so a single
        module-level slot is sufficient -- no thread-local storage needed.
        """
        global _ACTIVE
        previous = _ACTIVE
        _ACTIVE = self
        self.start_time = time.perf_counter()
        try:
            yield self
        finally:
            self.end_time = time.perf_counter()
            _ACTIVE = previous

    def record(self, name, elapsed):
        self.totals[name] = self.totals.get(name, 0.0) + elapsed
        self.counts[name] = self.counts.get(name, 0) + 1

    @property
    def total_time(self):
        if self.start_time is None:
            return sum(self.totals.values())
        end = self.end_time if self.end_time is not None else time.perf_counter()
        return max(0.0, end - self.start_time)

    def sorted_stages(self):
        return sorted(self.totals.items(), key=lambda kv: -kv[1])

    def report_lines(self):
        grand = self.total_time
        accounted = sum(self.totals.values())
        lines = []
        for name, t in self.sorted_stages():
            n = self.counts.get(name, 0)
            pct = (100.0 * t / grand) if grand > 1.0e-9 else 0.0
            per_call = (t / n * 1000.0) if n else 0.0
            lines.append(
                f"{name:<22s} {t * 1000.0:9.1f} ms  {pct:5.1f}%  "
                f"{n:6d} call(s)  {per_call:8.3f} ms/call"
            )
        other = max(0.0, grand - accounted)
        if other > 1.0e-6:
            pct = (100.0 * other / grand) if grand > 1.0e-9 else 0.0
            lines.append(f"{'(unaccounted)':<22s} {other * 1000.0:9.1f} ms  {pct:5.1f}%")
        lines.append(f"{'TOTAL':<22s} {grand * 1000.0:9.1f} ms")
        return lines

    def report_text(self, header=""):
        lines = []
        if header:
            lines.append(header)
        lines.extend(self.report_lines())
        return "\n".join(lines)

    def summary_text(self, max_stages=3):
        """One short line for Blender's status-bar report() calls."""
        grand = self.total_time
        top = self.sorted_stages()[:max_stages]
        parts = [f"{name} {t * 1000.0:.0f}ms" for name, t in top]
        return f"solve {grand * 1000.0:.0f}ms (" + ", ".join(parts) + ")"

    def as_dict(self):
        return {
            'total_ms': self.total_time * 1000.0,
            'stage_ms': {k: v * 1000.0 for k, v in self.totals.items()},
            'stage_calls': dict(self.counts),
        }


@contextmanager
def stage(name):
    """Time a block of solver work under `name` if a profile is active.

    No-op (single `if` check) when profiling is off, so this is safe to leave
    wrapped around hot paths permanently.
    """
    profile = _ACTIVE
    if profile is None:
        yield
        return
    start = time.perf_counter()
    try:
        yield
    finally:
        profile.record(name, time.perf_counter() - start)


def active_profile():
    return _ACTIVE


def mark():
    """Return a start timestamp for a loop iteration with multiple exit points.

    Use with `record_since()` when a `with stage(...):` block is awkward
    because the timed region has several `continue`/`break` exits, e.g. a
    search loop over solve attempts.
    """
    return time.perf_counter()


def record_since(name, start):
    """Record elapsed time since `mark()` under `name`, if profiling is active."""
    profile = _ACTIVE
    if profile is None:
        return
    profile.record(name, time.perf_counter() - start)


def timed(name):
    """Decorator form of `stage()` for timing an entire function call."""
    def decorator(func):
        def wrapper(*args, **kwargs):
            with stage(name):
                return func(*args, **kwargs)
        wrapper.__name__ = getattr(func, '__name__', name)
        wrapper.__doc__ = func.__doc__
        return wrapper
    return decorator
