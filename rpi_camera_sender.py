import socket
import struct
import cv2
from picamera2 import Picamera2

# Network configuration
HOST = "0.0.0.0"  # Listen on all available network interfaces
PORT = 8485

# Initialize camera
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (640, 480)}
)
picam2.configure(config)
picam2.start()

# Set up TCP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"Waiting for client connection on port {PORT}...")
conn, addr = server_socket.accept()
print(f"Connected to {addr}")

try:
  while True:
    # Capture frame from Pi camera
    frame = picam2.capture_array()

    # Compress frame to JPEG (quality 80 balances speed and clarity)
    _, encoded_frame = cv2.imencode(
        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80]
    )
    data = encoded_frame.tobytes()

    # Send frame size (8-byte unsigned long long) followed by the frame bytes
    conn.sendall(struct.pack(">Q", len(data)) + data)

except (BrokenPipeError, ConnectionResetError):
  print("Client disconnected.")
except KeyboardInterrupt:
  print("Stopping stream...")
finally:
  conn.close()
  server_socket.close()
  picam2.stop()
