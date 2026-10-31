
import wifi_communication
import controller
import time

if __name__ == "__main__":
    try:
        node = wifi_communication.UDPNode(broadcast_ip='255.255.255.255', send_port=5001, listen_port=5002, name="Control Hub")
        node.start_listening()
        while True:
            robot_controller = controller.controller()
            input = robot_controller.get_input()
            formatted_input = ", ".join(str(round(x, 2)) for x in input)
            print(formatted_input)
            node.broadcast(formatted_input)
            time.sleep(1)   
    except KeyboardInterrupt:
        print("Exiting. . .")