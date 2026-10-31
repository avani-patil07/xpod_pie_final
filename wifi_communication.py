import socket
import threading

class UDPNode:
    def __init__(self, broadcast_ip, send_port, listen_port, name):
        self.broadcast_ip = broadcast_ip
        self.send_port = send_port
        self.listen_port = listen_port
        self.name = name
        
        # 1. Set up the Receiver (Listening) Socket
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(('', self.listen_port))
        
        # 2. Set up the Sender (Broadcasting) Socket
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def _listen_loop(self):
        """Background loop that constantly waits for incoming messages."""
        while True:
            data, addr = self.rx.recvfrom(1024)
            print(f"\n[Received] From {addr}: {data.decode()}\n> ", end="")

    def start_listening(self):
        """Spins up the receiver in the background so it never blocks the main program."""
        threading.Thread(target=self._listen_loop, daemon=True).start()

    def broadcast(self, message):
        """Sends a single message out to the network on command."""
        full_message = f"{self.name}: {message}"
        self.tx.sendto(full_message.encode(), (self.broadcast_ip, self.send_port))

# --- How to use it ---
if __name__ == "__main__":
    # CONFIGURATION FOR DEVICE 1 (Swap 5001 and 5002 if running on Device 2!)
    node = UDPNode(broadcast_ip='255.255.255.255', send_port=5001, listen_port=5002, name="Device 1")
    
    # Start the background listener
    node.start_listening()
    print(f"--- {node.name} Ready. Type a message and press Enter to broadcast ---")

    # This loop lets YOU control when it broadcasts using your keyboard
    while True:
        user_input = input("> ")
        if user_input.strip():
            node.broadcast(user_input)
