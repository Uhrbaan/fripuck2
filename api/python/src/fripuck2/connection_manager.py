import selectors
import threading
import socket
from collections.abc import Callable

from .robot import Robot

class ConnectionManager:
    def __init__(self, udp_port: int=65431):
        self.udp_port = udp_port
        self.running = True
        
        self.lock = threading.Lock()
        self.selector = selectors.DefaultSelector() # to pack tcp sockets together
        self.registry = dict[str, tuple[socket.socket, Callable[[bytes], None]]]() # stores the callbacks and the sockets for each robot

        # Set up the shared UDP socket
        self.udp_ip = "0.0.0.0"
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind((self.udp_ip, self.udp_port))
        self.udp_sock.settimeout(1.0) # Prevent thread from hanging on close
        self.selector.register(self.udp_sock, selectors.EVENT_READ, data=(self.udp_ip, None))
        print("Bound UDP connection")
                
        self.thread = threading.Thread(target=self._listener, daemon=True)
        self.thread.start()

    # TODO: Add option to automatically try to run register_robot again if the connection is lost.
    def register_robot(self, iprobot, callback: Callable[[bytes], None] = None, tcp_port = 65430) -> Callable[[bytes], int]:
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

        # create a new TCP sockec for this robot
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setblocking(False)

        # try to connect to the robot
        try:
            sock.connect_ex((ip, tcp_port)) 
        except Exception as e:
            print(f"Initial connection to {ip} failed: {e}")

        with self.lock:
            # Register the socket to the selector wo we can watch it for read events with the other sockets.
            self.selector.register(sock, selectors.EVENT_READ, data=(ip, cb))
            self.registry[ip] = (sock, cb)
        
        # Return the send function of _this_ robot
        def send_func(data: bytes):
            """Function specific to this IP address, to send data over TCP."""
            try:
                sock.sendall(data)
            except BrokenPipeError:
                print(f"Connection to {ip} lost.")  
        
        print(f"Registered robot {ip}")
        return send_func
    
    def _listener(self):
        """Function that manages incoming traffic."""

        while self.running:
            # Wait for any socket to have data (1-second timeout)
            events = self.selector.select(timeout=1.0)
            for key, mask in events:
                ip, callback = key.data # unwrap ip-cb tuple
                sock = key.fileobj
                
                if sock == self.udp_sock:
                    data, addr = sock.recvfrom(4096)
                    self._handle_udp(data, addr)
                else:
                    try:
                        data = sock.recv(4096)
                        if data:
                            callback(data)
                        else:
                            self._unregister(ip, sock) # Robot probably disconnected if data is empty.
                    except Exception as e:
                        print(f"Error while receiving data: {e}")
                        self._unregister(ip, sock)

    def _handle_udp(self, data, addr):
        """Handle incoming UDP data, send it to the right robot instance."""
        ip = addr[0]
        with self.lock:
            if ip in self.registry:
                _, callback = self.registry[ip]
                callback(data)
    
    def _unregister(self, ip: str, sock: socket.socket):
        with self.lock:
            try:
                self.selector.unregister(sock)
                if ip in self.registry:
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
