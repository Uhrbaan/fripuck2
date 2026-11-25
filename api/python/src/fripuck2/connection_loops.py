"""Background threads for :class:`~fripuck2.connection_manager.ConnectionManager`.

Kept separate from the registry code because these methods are long-running
loops: they own the reconnect queue and the per-socket read events, and are the
only code that runs off the caller's thread.
"""

import queue
import selectors
import socket
from collections.abc import Callable


class ConnectionLoops:
    """See module docstring."""

    def _reconnect_pending(self):
        """Function that will try to reconnect a robot."""
        try:
            to_be_connected = self.reconnect_queue.get(block=False)
            ip, cb, on_connect = to_be_connected
            try:
                self._reconnect(ip, cb, on_connect)
            except Exception as e:
                print(f"Was unable to (re)connect robot {ip} because of {e}.")
                # place the robot on the back of the queue 
                self.reconnect_queue.put((ip, cb, on_connect))    
        except queue.Empty:
            # If there is nothing to reconnect, just continue.
            pass 

    def _reconnect(self, ipv4addr: str, callback: Callable[[bytes], None],
                   on_connect: Callable[[bool], None] = None):
        # create a new TCP sockec for this robot
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setblocking(True) # TODO: change this for a more complex non-blocking method.

        # Try to detect if the connection was lost (e.g. the robot turned off (slow))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)

        try: 
            sock.connect((ipv4addr, self.tcp_port)) 
            print(f"Successfully connected robot {ipv4addr}!")
        except Exception as e:
            print(f"Could not connect to {ipv4addr} because of: {e}")
            raise e

        with self.lock:
            # Register the socket to the selector wo we can watch it for read events with the other sockets.
            self.selector.register(sock, selectors.EVENT_READ, data=(ipv4addr, callback))
            self.registry[ipv4addr] = (sock, callback, on_connect)

        if callable(on_connect):
            on_connect(True)

    def _udp_task(self):
        """Manages the UDP connection for all the robots."""
        while self.running:
            try:
                # This blocks for up to 1 second
                data, addr = self.udp_sock.recvfrom(1500)

                # handle data
                ip = addr[0]
                with self.lock: # locking because we don't want to change the registry while it is being read
                    if ip in self.registry: 
                        _, callback, _ = self.registry[ip]
                        callback(data) # the callback should technically only unpack the flatbuffers, which is very efficient.

            except socket.timeout:
                # This can happen and is expected.
                continue 
            except Exception as e:
                print(f"Unexpected error: {e}")
                break
        self.udp_sock.close()

    def _tcp_task(self):
        """Function that manages incoming traffic."""
        while self.running:
            self._reconnect_pending() # First, try to reconnect any disconnected robots
            events = self.selector.select(timeout=1.0) # Wait for any socket to have data (1-second timeout)
            for key, mask in events:
                ip, callback = key.data # unwrap ip-cb tuple
                sock = key.fileobj

                try:
                    data = sock.recv(4096)
                    if data:
                        callback(data)
                    else:
                        raise Exception("Data is empty.")
                except Exception as e:
                    print(f"Error while receiving data: {e}")
                    self._unregister(ip, sock)
