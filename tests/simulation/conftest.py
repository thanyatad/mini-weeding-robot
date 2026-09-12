"""Isaac Sim's lifecycle, owned in one place for every test under tests/simulation.

Two things make this awkward enough to be worth a conftest rather than a
fixture per module:

1. Isaac allows exactly one ``SimulationApp`` per process, so the app has to be
   shared by every module in the directory, not built per module.
2. Isaac's standalone launcher runs with ``--/app/fastShutdown=True``, so
   ``SimulationApp.close()`` takes the process down immediately.  Close it
   anywhere before pytest has finished reporting - a fixture teardown, or even
   ``pytest_sessionfinish``, which ties with the terminal reporter's own hook -
   and the summary line never prints.  The tests all run and the exit code is
   still right, but the output stops at the progress line, which reads exactly
   like a crash.  ``pytest_unconfigure`` is the last hook of the session and is
   the only one reliably after the reporter.

Everything here is a no-op when Isaac is not installed, so ``pytest -q`` at the
repo root stays fast and GPU-free.
"""

_app = None


def isaac_app():
    """The one SimulationApp, started on first use."""
    global _app
    if _app is None:
        from isaacsim import SimulationApp

        _app = SimulationApp({"headless": True})
    return _app


def pytest_unconfigure(config):
    """Close Isaac after the reporter has had its say, not before."""
    global _app
    if _app is not None:
        app, _app = _app, None
        app.close()
