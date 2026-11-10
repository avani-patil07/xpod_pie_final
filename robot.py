
import wifi_communication
import controller

if __name__ == "__main__":
    try:
        node = wifi_communication.UDPNode(broadcast_ip='192.168.1.168', send_port=5002, listen_port=5001, name="Robot")
        node.start_listening()
        while True:
            pass
    except KeyboardInterrupt():
        print("Exiting. . .")