"""Registry of robot connections and the TCP/UDP sockets behind them.

The public surface (:meth:`register_ip`, :meth:`send_tcp`, :meth:`shutdown`)
lives here; the background receive loops live in ``connection_loops.py``.
"""

import queue
import selectors
import socket
import threading
from collections.abc import Callable

from .connection_loops import ConnectionLoops


class ConnectionManager(ConnectionLoops):
    """See module docstring."""

    def __init__(self, tcp_port:int = 65430, udp_port: int=65431):
        self.tcp_port = tcp_port
        self.udp_port = udp_port
        self.running = True

        self.lock = threading.Lock()
        self.selector = selectors.DefaultSelector() # to pack tcp sockets together
        # ip -> (tcp socket, packet callback, on_connect callback)
        self.registry: dict[str, tuple[socket.socket, Callable[[bytes], None], Callable[[bool], None] | None]] = {}
        self.reconnect_queue = queue.Queue()

        # Set up the shared UDP socket
        self.udp_ip = "0.0.0.0"
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind((self.udp_ip, self.udp_port))
        self.udp_sock.settimeout(1.0) # Prevent thread from hanging on close
        self.udp_thread = threading.Thread(target=self._udp_task, daemon=True)
        self.udp_thread.start()
        print("Bound UDP connection")

        self.thread = threading.Thread(target=self._tcp_task, daemon=True)
        self.thread.start()

    def register_ip(self, ip: str, callback: Callable[[bytes], None],
                    on_connect: Callable[[bool], None] = None) -> Callable[[bytes | str], None]:
        """Registers the robot to the connection manager. 

        :param ip: IP address of the robot.
        :param callback: called with every packet (TCP or UDP) received from the robot.
        :param on_connect: optional callback called with True when the TCP connection 
            is established, and False when it is lost. Useful to keep a 'connected' flag.
        """
        print(f"Attempting to connect to robot {ip}...")

        # Keep trying to connect until successful or the manager is shut down
        # connected = False
        # while not connected and self.running:
        #     try:
        #         self._reconnect(ip, callback)
        #         connected = True
        #     except Exception as e:
        #         print(f"Failed to connect to {ip} ({e}). Retrying in 1 second...")
        #         time.sleep(1)

        # print(f"Registered robot {ip}")

        self.reconnect_queue.put((ip, callback, on_connect))

        # Return the function so the robot can send tcp data
        return lambda data: self.send_tcp(ip=ip, data=data)

    def send_tcp(self, ip: str, data: bytes | str) -> bool:
        """
        Sends binary data to a registered robot over TCP.
        Returns True if successful, False if the robot is disconnected or the send failed.
        """
        sock = None

        with self.lock:
            if ip in self.registry:
                sock, _, _ = self.registry[ip]

        if not sock:
            print(f"Cannot send: Robot {ip} is not currently connected.")
            return False

        try:
            sock.sendall(data) # ensure the entire payload is sent
            return True
        except Exception as e:
            print(f"Failed to send data to {ip}: {e}")
            self._unregister(ip, sock)
            return False

    def _unregister(self, ip: str, sock: socket.socket):
        with self.lock:
            try:
                self.selector.unregister(sock)
                if ip in self.registry:
                    _, callback, on_connect = self.registry[ip]
                    self.reconnect_queue.put((ip, callback, on_connect)) # send it to the queue to be reconnected when the robot comes online again
                    del self.registry[ip]
                    if callable(on_connect):
                        on_connect(False)
                sock.close()
                print(f"Cleanly unregistered {ip}")
            except Exception as e:
                print(f"Error during unregistering of {ip}: {e}")

    def shutdown(self):
        """Shutdown Connection manager. Closes the sockets to all the robots."""
        self.running = False
        for thread in (self.thread, self.udp_thread):
            if thread.is_alive():
                thread.join()

        # Cleanup all remaining sockets. _unregister() re-queues robots for
        # reconnection, so stop that first and close the UDP socket directly:
        # the UDP thread already closed it on exit, and it was never registered
        # with the selector (that would raise ValueError on an invalid fd).
        with self.lock:
            closed_ips = list(self.registry)
            for ip in closed_ips:
                sock, _, _ = self.registry.pop(ip, (None, None, None))
                if sock is not None:
                    try:
                        self.selector.unregister(sock)
                    except (KeyError, ValueError):
                        pass
                    sock.close()
                    print(f"Cleanly unregistered {ip}")
            self.reconnect_queue = queue.Queue()  # drop pending reconnects
            self.udp_sock.close()
            self.selector.close()