"""LED control.

``LedMixin`` maps the many ways a student may name an LED (``1``, ``"ring_1"``,
``"led1"``, ``"front"``, ``"all"``, ...) onto the firmware's ``LedTarget``
enum, and normalises the accepted value forms (bool, int, RGB, RGB+intensity).
"""

from ._generated import commands_generated as commands


class LedMixin:
    """See module docstring."""

    _LED_TARGET_MAP = {
        1: commands.LedTarget.RING_1,
        "1": commands.LedTarget.RING_1,
        "ring1": commands.LedTarget.RING_1,
        "ringled1": commands.LedTarget.RING_1,
        "led1": commands.LedTarget.RING_1,

        2: commands.LedTarget.RING_2,
        "2": commands.LedTarget.RING_2,
        "ring2": commands.LedTarget.RING_2,
        "ringled2": commands.LedTarget.RING_2,
        "led2": commands.LedTarget.RING_2,

        3: commands.LedTarget.RING_3,
        "3": commands.LedTarget.RING_3,
        "ring3": commands.LedTarget.RING_3,
        "ringled3": commands.LedTarget.RING_3,
        "led3": commands.LedTarget.RING_3,

        4: commands.LedTarget.RING_4,
        "4": commands.LedTarget.RING_4,
        "ring4": commands.LedTarget.RING_4,
        "ringled4": commands.LedTarget.RING_4,
        "led4": commands.LedTarget.RING_4,

        5: commands.LedTarget.RING_5,
        "5": commands.LedTarget.RING_5,
        "ring5": commands.LedTarget.RING_5,
        "ringled5": commands.LedTarget.RING_5,
        "led5": commands.LedTarget.RING_5,

        6: commands.LedTarget.RING_6,
        "6": commands.LedTarget.RING_6,
        "ring6": commands.LedTarget.RING_6,
        "ringled6": commands.LedTarget.RING_6,
        "led6": commands.LedTarget.RING_6,

        7: commands.LedTarget.RING_7,
        "7": commands.LedTarget.RING_7,
        "ring7": commands.LedTarget.RING_7,
        "ringled7": commands.LedTarget.RING_7,
        "led7": commands.LedTarget.RING_7,

        8: commands.LedTarget.RING_8,
        "8": commands.LedTarget.RING_8,
        "ring8": commands.LedTarget.RING_8,
        "ringled8": commands.LedTarget.RING_8,
        "led8": commands.LedTarget.RING_8,

        "body": commands.LedTarget.BODY,
        "bodyled": commands.LedTarget.BODY,

        "front": commands.LedTarget.FRONT,
        "frontled": commands.LedTarget.FRONT,

        "all": commands.LedTarget.ALL,
    }

    @classmethod
    def _resolve_led_target(cls, key):
        norm = key.lower().replace("_", "") if isinstance(key, str) else key
        if norm not in cls._LED_TARGET_MAP:
            raise ValueError(f"Unknown LED identifier: {key!r}")
        return cls._LED_TARGET_MAP[norm]

    @staticmethod
    def _normalize_led_value(value):
        if isinstance(value, bool):
            return value, ((255, 255, 255) if value else (0, 0, 0)), 100

        if isinstance(value, int):
            is_on = value != 0
            return is_on, ((255, 255, 255) if is_on else (0, 0, 0)), 100

        if isinstance(value, (tuple, list)) and len(value) in (3, 4):
            r = max(0, min(255, int(value[0])))
            g = max(0, min(255, int(value[1])))
            b = max(0, min(255, int(value[2])))
            intensity = max(0, min(100, int(value[3]))) if len(value) == 4 else 100
            is_on = (intensity > 0) and (r > 0 or g > 0 or b > 0)
            return is_on, (r, g, b), intensity

        raise ValueError(f"Invalid LED value {value!r}: expected bool, int, or (r, g, b[, intensity])")

    def set_leds(self, leds: dict = None, fade_ms: int = 0, **kwargs):
        """Sets multiple LEDs on the robot in a single command.

        Accepts a dictionary mapping LED identifiers (ring numbers ``1``..``8``,
        ``"ring_1"``..``"ring_8"``, ``"body"``, ``"front"``, or ``"all"``) and/or
        keyword arguments to values:

        - ``bool`` or ``int`` (``0``/``1``): turns binary LEDs on/off, or sets RGB
          LEDs to full white / off.
        - ``(r, g, b)`` or ``(r, g, b, intensity)`` tuple/list: sets RGB color
          (``0..255``) and optional brightness percentage (``0..100``). Any
          non-zero color turns binary LEDs on; ``(0, 0, 0)`` turns them off.

        :param leds: Optional mapping of LED keys to values, e.g.
            ``{1: True, 2: (255, 0, 0), "front": True}``.
        :param fade_ms: Transition duration in milliseconds for RGB LEDs
            (``0`` = immediate).
        :param kwargs: Optional named LEDs, e.g. ``front=True, body=False``.

        Example::

            robot.set_leds({1: True, 2: (255, 0, 0), 4: (0, 255, 0)}, fade_ms=200)
            robot.set_leds(front=True, body=True)
        """
        combined = {}
        if leds:
            combined.update(leds)
        if kwargs:
            combined.update(kwargs)
        if not combined:
            return

        entries = []
        for key, val in combined.items():
            target = self._resolve_led_target(key)
            is_on, rgb, intensity = self._normalize_led_value(val)
            entries.append({
                "target": target,
                "on": is_on,
                "color": rgb,
                "intensity": intensity,
            })

        self._queue_or_send({
            "type": "SetLeds",
            "entries": entries,
            "fade_ms": max(0, int(fade_ms)),
        })

    def set_led(self, target, value, g: int = None, b: int = None, intensity: int = 100, fade_ms: int = 0):
        """Sets a single LED (or ``"all"``) on the robot.

        Supports both Lua-style tuple/boolean calls and explicit RGB arguments::

            robot.set_led("all", False)
            robot.set_led(1, True)
            robot.set_led(2, (255, 0, 0), fade_ms=300)
            robot.set_led(2, 255, 0, 0, intensity=80, fade_ms=300)
            robot.set_led("front", True)
        """
        if g is not None and b is not None:
            resolved = (int(value), int(g), int(b), int(intensity))
        else:
            resolved = value

        self.set_leds({target: resolved}, fade_ms=fade_ms)
