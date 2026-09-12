"""A byte-level check on tracked text files.

This exists because of a real bug rather than a hypothetical one.  The two
``\\b`` escapes in ``test_layering.py`` were once collapsed into literal 0x08
backspace bytes on the way into the file, which turned a guard on the control
path into a pattern that could never match.  Nothing caught it: the line reads
correctly in an editor, ``ruff`` is clean, and the test was green precisely
because it had stopped testing anything.

A terminal hides these bytes by rendering them, so the only way to see the
corruption is to look at the bytes.  That is all this does.
"""

import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]

#: Control characters that have no business in source.  Tab (0x09), newline
#: (0x0a) and carriage return (0x0d) are deliberately absent — they are how
#: text files are written, and CRLF is normal on the Windows side of this repo.
FORBIDDEN_BYTES = frozenset(range(0x00, 0x09)) | {0x0B, 0x0C} | frozenset(range(0x0E, 0x20))


def _tracked_files() -> list[Path]:
    """Every file git knows about, which is the set worth policing.

    Anything untracked is scratch work and not the repo's problem.
    """
    try:
        result = subprocess.run(
            ["git", "ls-files", "-z"],
            cwd=REPO,
            capture_output=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        pytest.skip(f"git is not usable here, cannot enumerate tracked files: {exc}")
    names = result.stdout.decode("utf-8").split("\0")
    return [REPO / name for name in names if name]


def test_no_tracked_text_file_contains_a_stray_control_character():
    """Catch the corruption that made ``test_layering.py`` vacuous.

    Files that are not valid UTF-8 are not text and are skipped, so adding an
    image or a binary fixture later does not require touching this test.
    """
    offenders = []
    for path in _tracked_files():
        raw = path.read_bytes()
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            continue
        for n, line in enumerate(text.splitlines(), 1):
            found = sorted({hex(ord(ch)) for ch in line if ord(ch) in FORBIDDEN_BYTES})
            if found:
                offenders.append(f"{path.relative_to(REPO).as_posix()}:{n} {found}")

    assert offenders == [], (
        "stray control characters in tracked text files — these survive review "
        f"because a terminal renders them away: {offenders}"
    )
