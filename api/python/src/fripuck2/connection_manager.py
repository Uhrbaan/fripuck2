import selectors
import threading
import socket
from collections.abc import Callable
import queue

from .robot import Robot

class ConnectionManager:
    def __init__(self, tcp_port:int = 65430, udp_port: int=65431):
        self.tcp_port = tcp_port
        self.udp_port = udp_port
        self.running = True
        
        self.lock = threading.Lock()
        self.selector = selectors.DefaultSelector() # to pack tcp sockets together
        self.registry = dict[str, tuple[socket.socket, Callable[[bytes], None]]]() # stores the callbacks and the sockets for each robot
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

    # TODO: Add option to automatically try to run register_robot again if the connection is lost.
    def register_robot(self, iprobot, callback: Callable[[bytes], None] = None) -> Callable[[bytes], int]:
        """Registers the robot to the connection manager. 

        You may provide an IP address AND a callback function, OR a single instance of Robot.
        
        :param iprobot: IPv4 address of the robot we want to connect to OR the robot instance iteself.
        :param callback: Callback function that will be called when we recieve a udp or tcp packet from that robot. It contains the data being sent as a parameter.
        :param tcp_port: Optional. Should only be changed if you use other firmware that does not use the port 65430.
        :return: A function that can be used to send data to the robot.
        """
        ip = ""
        cb = None
        if type(iprobot) is str: 
            ip = iprobot 
            cb = callback
        if isinstance(iprobot, Robot):
            ip = iprobot.ip
            cb = iprobot._receive_packet
        else:
            # raise a type error if the 
            raise TypeError

        # Add the robot to the reconnect queue so it gets connected shortly.
        self.reconnect_queue.put((ip, cb))
        print(f"Registered robot {ip}")

    def _reconnect_pending(self):
        """Function that will try to reconnect a robot."""
        try:
            to_be_connected = self.reconnect_queue.get(block=False)
            ip, cb = to_be_connected
            try:
                self._reconnect(ip, cb)
            except Exception as e:
                print(f"Was unable to (re)connect robot {ip} because of {e}.")
                # place the robot on the back of the queue 
                self.reconnect_queue.put((ip, cb))    
        except queue.Empty:
            # If there is nothing to reconnect, just continue.
            pass 
    
    def _reconnect(self, ipv4addr: str, callback: Callable[[bytes], None]):
        # create a new TCP sockec for this robot
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setblocking(True) # TODO: change this for a more complex non-blocking method.

        # Try to detect if the connection was lost (e.g. the robot turned off (slow))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)

        try: 
            sock.connect_ex((ipv4addr, self.tcp_port)) 
            print(f"Successfully connected robot {ipv4addr}!")
        except Exception as e:
            print(f"Could not connect to {ipv4addr} because of: {e}")
            raise e

        with self.lock:
            # Register the socket to the selector wo we can watch it for read events with the other sockets.
            self.selector.register(sock, selectors.EVENT_READ, data=(ipv4addr, callback))
            self.registry[ipv4addr] = (sock, callback)
    
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
                        _, callback = self.registry[ip]
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
    
    def _unregister(self, ip: str, sock: socket.socket):
        with self.lock:
            try:
                self.selector.unregister(sock)
                if ip in self.registry:
                    _, callback = self.registry[ip]
                    self.reconnect_queue.put((ip, callback)) # send it to the queue to be reconnected when the robot comes online again
                    del self.registry[ip]
                sock.close()
                print(f"Cleanly unregistered {ip}")
            except Exception as e:
                print(f"Error during unregistering of {ip}: {e}")

    def shutdown(self):
        """Shutdown Connection manager. Closes the sockets to all the robots."""
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        
        # Cleanup all remaining sockets
        with self.lock:
            for ip, (sock, _) in list(self.registry.items()): # ignore cb
                self._unregister(ip, sock)
            self.selector.unregister(self.udp_sock)
            self.udp_sock.close()
            self.selector.close()
