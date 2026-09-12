"""seq -> sent_at, and link_age_ms measured from the last_seq the board echoes.

The point of this module is that it needs no clock sync: the Pi stamps the
send with its own clock and subtracts with the same clock when the echo comes
back. The ESP32 contributes a number it copied, nothing more -- its uptime_ms
is never read here, and a board whose clock is wrong by an hour changes none of
these answers.

The second thing under test is memory. At 10 Hz, forever, a table that never
forgets is a leak in the one process that has to survive a whole field session.
"""

import pytest

from bridge.seq_tracker import SeqTracker, UnknownSeq
from controller.config import get, load_config, with_overrides

LINK_LOST_MS = 500.0


@pytest.fixture
def tracker():
    return SeqTracker(retention_ms=LINK_LOST_MS)


def test_age_is_the_pi_clock_difference_between_send_and_echo(tracker):
    tracker.sent(101, now_ms=1000.0)
    tracker.sent(102, now_ms=1100.0)

    assert tracker.age(102, now_ms=1150.0) == 50.0
    assert tracker.age(101, now_ms=1150.0) == 150.0


def test_the_board_clock_is_never_consulted(tracker):
    """uptime_ms exists to spot a reboot, not to measure latency: the two
    clocks are never compared, so there is nothing to synchronise."""
    tracker.sent(1, now_ms=0.0)

    # Whatever the board thinks the time is, the answer is the Pi's own.
    assert tracker.age(1, now_ms=120.0) == 120.0


def test_a_dropped_frame_costs_nothing(tracker):
    """The correlation example in protocol/messages.md: seq 103 goes missing,
    104 arrives 100 ms later, and link age never approaches link_lost_ms."""
    for seq, sent_at in ((101, 0.0), (102, 100.0), (103, 200.0), (104, 300.0)):
        tracker.sent(seq, now_ms=sent_at)

    # The board never saw 103, so it echoes 102 and then 104.
    assert tracker.age(102, now_ms=250.0) == 150.0
    assert tracker.age(104, now_ms=350.0) == 50.0
    assert tracker.age(104, now_ms=350.0) < LINK_LOST_MS


def test_seq_must_climb(tracker):
    """Monotonic per session (Rule 3). A seq that repeats would overwrite the
    send time of a frame still in flight."""
    tracker.sent(5, now_ms=0.0)

    with pytest.raises(ValueError, match="monotonic"):
        tracker.sent(5, now_ms=100.0)
    with pytest.raises(ValueError, match="monotonic"):
        tracker.sent(4, now_ms=100.0)


def test_a_seq_that_was_never_sent_is_rejected(tracker):
    """A board echoing a seq ahead of anything the Pi sent is a corrupted line
    or a different session -- not a link age of zero."""
    tracker.sent(5, now_ms=0.0)

    with pytest.raises(UnknownSeq):
        tracker.age(6, now_ms=100.0)


def test_nothing_sent_yet_has_no_age(tracker):
    with pytest.raises(UnknownSeq):
        tracker.age(0, now_ms=100.0)


def test_a_board_that_has_echoed_nothing_ages_from_the_first_send(tracker):
    """last_seq is 0 out of reset, before any frame of this session reached the
    board. The link has been unconfirmed since the first thing the Pi sent, and
    that is what the age has to say -- not zero."""
    tracker.sent(1, now_ms=100.0)
    tracker.sent(2, now_ms=200.0)

    assert tracker.age(0, now_ms=300.0) == 200.0


def test_memory_is_bounded(tracker):
    """10 000 sends at 10 Hz is a little over 16 minutes of driving. Keeping
    link_lost_ms worth of history is all anything asks for."""
    for seq in range(1, 10_001):
        tracker.sent(seq, now_ms=seq * 100.0)

    assert tracker.tracked <= 1 + LINK_LOST_MS / 100.0
    assert tracker.tracked >= 2, "a tracker that keeps nothing cannot answer anything"


def test_eviction_never_reports_a_forgotten_frame_as_recent():
    """The dangerous direction, and the reason for the horizon.

    Three seconds pass with nothing sent -- the process was busy, or the port
    was gone -- and then the board comes back echoing the last seq it ever saw.
    Aging that seq against the oldest entry still in the table would answer
    0 ms, and link_lost would never trip on a link that has been dead for three
    seconds.
    """
    tracker = SeqTracker(retention_ms=LINK_LOST_MS)
    for seq in range(1, 6):
        tracker.sent(seq, now_ms=seq * 100.0)

    tracker.sent(100, now_ms=3500.0)

    assert tracker.tracked == 1, "the gap should have emptied the window"
    assert tracker.age(3, now_ms=3500.0) >= LINK_LOST_MS


def test_retention_comes_from_link_lost_ms():
    config = load_config()
    assert SeqTracker.from_config(config).retention_ms == get(config, "safety.link_lost_ms")

    stretched = with_overrides(config, {"safety.link_lost_ms": 2000})
    assert SeqTracker.from_config(stretched).retention_ms == 2000


def test_next_seq_climbs_without_recording_a_send(tracker):
    """Allocation and the send stamp are two steps because the line goes out
    between them: the time that matters is the time it was written."""
    assert tracker.next_seq() == 1
    assert tracker.next_seq() == 1, "allocating twice without sending is still seq 1"

    tracker.sent(1, now_ms=0.0)
    assert tracker.next_seq() == 2
    assert tracker.tracked == 1
