"""seq -> sent_at, and link_age_ms computed from the last_seq the board echoes.

    tracker.sent(seq, now_ms)
    link_age_ms = tracker.age(state["last_seq"], now_ms)

No clock sync between the Pi and the ESP32, and none needed (§8.3): the Pi
stamps the send with its own clock and subtracts with the same clock when the
echo arrives. The board copies a number back, which is all it is asked to do.
Its ``uptime_ms`` is never read here -- that field exists to spot a reboot, and
a board whose clock is an hour out changes none of these answers.

This holds no clock of its own, like the state machine and the row-loss
watchdog: ``now_ms`` arrives from the caller, which is what makes a synthetic
timeline reproducible to the millisecond. The one place a real clock enters the
system is the rover backend that owns the loop.

Eviction
--------
At 10 Hz a table that never forgets grows by ten entries a second for the
length of a field session. Only ``retention_ms`` (= ``safety.link_lost_ms``) of
history is ever asked about: past that the answer is the same either way, since
the link monitor has already tripped.

What eviction must not do is make a forgotten frame look recent. A seq that has
fallen out of the window is aged against the horizon -- the send time of the
newest entry dropped -- which by construction is already older than
``retention_ms``. Aging it against the oldest surviving entry instead would let
a link that died half a minute ago report 400 ms, and ``link_lost`` would never
trip.
"""

from __future__ import annotations

from collections import deque
from typing import Any


class UnknownSeq(Exception):
    """The board echoed a ``last_seq`` this session never sent.

    A corrupted line or a board still running the previous session -- either
    way it is not a link age of zero, which is what silently returning would
    amount to.
    """


class SeqTracker:
    """Send times for streaming commands, bounded by ``retention_ms``."""

    def __init__(self, retention_ms: float) -> None:
        if retention_ms <= 0:
            raise ValueError(
                f"retention_ms must be positive, got {retention_ms} — "
                f"a tracker that keeps nothing can never measure a link age"
            )
        self.retention_ms = float(retention_ms)
        self._sent_at: dict[int, float] = {}
        self._order: deque[int] = deque()
        self._highest_sent: int | None = None
        #: Send time of the newest frame no longer resolvable exactly.  Starts
        #: at the first send, so a board that has echoed nothing yet ages from
        #: there, and advances with every eviction.
        self._horizon_ms: float | None = None

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> SeqTracker:
        from controller.config import get

        return cls(retention_ms=get(config, "safety.link_lost_ms"))

    @property
    def tracked(self) -> int:
        """How many send times are being kept right now."""
        return len(self._sent_at)

    @property
    def highest_sent(self) -> int | None:
        return self._highest_sent

    def next_seq(self) -> int:
        """The seq the next frame should carry.  Records nothing.

        Allocating and stamping are two steps because the line goes out between
        them, and the time worth keeping is the time it was written.
        """
        return 1 if self._highest_sent is None else self._highest_sent + 1

    def sent(self, seq: int, now_ms: float) -> None:
        """Record that ``seq`` went out at ``now_ms``, then forget what is stale."""
        if self._highest_sent is not None and seq <= self._highest_sent:
            raise ValueError(
                f"seq must be monotonic per session: {seq} follows {self._highest_sent}"
            )

        self._sent_at[seq] = float(now_ms)
        self._order.append(seq)
        self._highest_sent = seq
        if self._horizon_ms is None:
            self._horizon_ms = float(now_ms)

        self._evict(float(now_ms))

    def age(self, last_seq: int, now_ms: float) -> float:
        """Milliseconds since the frame the board is echoing was sent.

        Raises :class:`UnknownSeq` for a seq this session never sent, which
        includes every seq while nothing has been sent at all.
        """
        if self._highest_sent is None or last_seq > self._highest_sent:
            raise UnknownSeq(
                f"the board echoed last_seq={last_seq}, which this session never sent "
                f"(highest sent: {self._highest_sent})"
            )

        sent_at = self._sent_at.get(last_seq)
        if sent_at is None:
            # Evicted, or from before this session's first frame reached the
            # board.  Either way the link has been unconfirmed since at least
            # the horizon.
            assert self._horizon_ms is not None
            sent_at = self._horizon_ms

        return float(now_ms) - sent_at

    def _evict(self, now_ms: float) -> None:
        """Drop what is older than the retention window.

        Strictly older: an entry at exactly ``retention_ms`` is the one the
        link monitor trips on, and it has to still be answerable then.
        """
        while self._order:
            oldest = self._order[0]
            if now_ms - self._sent_at[oldest] <= self.retention_ms:
                return
            self._horizon_ms = self._sent_at.pop(oldest)
            self._order.popleft()
