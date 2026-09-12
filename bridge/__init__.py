"""Transport and encoding between the controller and the board.

No business logic lives here (bridge/README.md): the controller decides, this
package carries. Two files, because the protocol has two halves that obey
different rules -- ``protocol.py`` for encode/decode and the discrete pending
table, ``seq_tracker.py`` for the streaming seq echo that link age is measured
from.

Imports are left to the caller so that ``simulator`` -- the ESP32 emulator,
which is test scaffolding -- is never pulled in by importing the codec.
"""
