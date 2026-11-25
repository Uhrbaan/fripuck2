"""The :class:`Robot` facade.

Assembles the per-concern mixins into the single object that students use.
Only state that several mixins share lives here: identity (``ip``), the
connection flag, the acceleration defaults, the sensor buffers and the
command queue.
"""

from .leds import LedMixin
from .motion import MotionMixin
from .peripherals import PeripheralMixin
from .protocol import ProtocolMixin
from .sensors import TofHisory, ProxHisory, ImuHistory, GroundHistory, AudioHistory
from .transport import TransportMixin


class Robot(TransportMixin, ProtocolMixin, MotionMixin, LedMixin, PeripheralMixin):
    """Control handle for one e-puck2 robot running the fripuck2 firmware.
    
    The class has two independent halves:
    
    **Commands (you -> robot).** ``drive``, ``move``, ``turn``, ``arc`` and
    ``set_wheels`` plan a motion; ``set_modules`` powers hardware on or off;
    ``play`` makes sound; ``upload_file`` pushes files to the robot's
    filesystem. All of these are fire-and-forget unless you wrap them in
    :meth:`batch`, and they do nothing until :meth:`connect` has been called and
    ``connected`` is ``True``.
    
    **Sensors (robot -> you).** Each sensor fills a rolling buffer that you read
    with ``get()`` (latest value) or ``get_all()`` (the whole window, as
    ``values, timestamps``). Data arrives in the background, so reads are
    instantaneous and never block on the network::
    
        distance = robot.tof.get()
        values, timestamps = robot.tof.get_all()
    
    Units follow SI conventions unless stated otherwise: metres, metres per
    second, metres per second squared and radians for motion; millimetres for
    ToF distance; seconds for timestamps.
    
    Where things live
    -----------------
    ``Robot`` is assembled from mixins, one file per concern, so keep this map
    handy when you go looking for a method:
    
    ============================  =========================================
    File                          Methods
    ============================  =========================================
    ``transport.py``              ``connect``
    ``protocol.py``               ``batch``, ``custom``
    ``motion.py``                 ``drive``, ``move``, ``turn``, ``arc``,
                                  ``set_wheels``, ``movement_settings``
    ``leds.py``                   ``set_led``, ``set_leds``
    ``peripherals.py``            ``set_modules``, ``upload_file``, ``play``
    ``robot.py`` (this file)      ``__init__``, ``connected``, sensor buffers
    ============================  =========================================
    
    Example::
    
        from fripuck2 import ConnectionManager, Robot
    
        cm = ConnectionManager()
        robot = Robot("192.168.2.214")
        robot.connect(cm)
    
        robot.set_modules(tof=True, motors=True)
        robot.move(0.5, 0.2)
    
        while robot.connected:
            print(robot.tof.get())
    
        cm.shutdown()
    """

    WHEEL_DIAMETER_M = 0.041
    WHEEL_DISTANCE_M = 0.053

    def __init__(self, ip: str, history_size=100): 
        """Creates a Robot handle for one e-puck2 running the fripuck2 firmware.

        Nothing is sent to the robot here: call :meth:`connect` to start the
        connection. The sensor buffers are created immediately, so you can read
        them before a connection exists (they simply stay empty).

        :param ip: IPv4 address of the robot, e.g. ``"192.168.2.214"``. It is
            also the address the robot sends its UDP sensor packets from, so it
            is used to tell robots apart when several are registered.
        :param history_size: Number of samples kept per sensor buffer. This is
            the maximum depth of the rolling window served by
            ``robot.<sensor>.get_all()``. Older samples are dropped, so pick a
            size that covers the longest window you want to inspect (e.g. `1000`
            at ~50 Hz keeps roughly the last 20 seconds).

        The following attributes are available after construction:

        ``ip``
            The robot's address, as passed in.
        ``connected``
            ``True`` while the TCP link to the robot is up, ``False`` before the
            connection is established and whenever it is lost. Poll this to know
            whether the robot is currently reachable: it is updated in the
            background and by the automatic reconnect logic.
        ``tof``, ``prox``, ``imu``, ``ground``, ``audio``
            Per-sensor history buffers. Read the latest sample with ``.get()`` or
            the whole window with ``values, timestamps = .get_all()``.
        ``default_accel``, ``default_decel``
            Default acceleration / deceleration in m/s^2, used by the movement
            methods when their ``accel`` / ``decel`` argument is left as ``None``.
            Change them in one go with :meth:`movement_settings`.

        Example::

            robot = Robot("192.168.2.214", history_size=1000)
            robot.connect(ConnectionManager())
            while not robot.connected:
                time.sleep(0.1)
        """        
        self.default_accel = 0.5
        self.default_decel = 0.5
        self._command_queue = None

        # True while the TCP connection to the robot is up. Set to False until
        # the connection is established and whenever it is lost.
        self.connected = False

        self.ip = ip 
        self.tof = TofHisory(size=history_size)
        self.prox = ProxHisory(size=history_size)
        self.imu = ImuHistory(size=history_size)
        self.ground = GroundHistory(size=history_size)
        self.audio = AudioHistory(size=history_size)
