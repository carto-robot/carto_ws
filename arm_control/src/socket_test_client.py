import socket
import time

def parse_pose(data):
    """
    Parse the received data into position and orientation components.

    :param data: Comma-separated string of pose data.
    :return: A dictionary with position and orientation.
    """
    try:
        x, y, z, qx, qy, qz, qw = map(float, data.split(','))
        pose = {
            'position': {
                'x': x,
                'y': y,
                'z': z,
            },
            'orientation': {
                'x': qx,
                'y': qy,
                'z': qz,
                'w': qw,
            }
        }
        return pose
    except ValueError as e:
        print(f"Error parsing data: {e}")
        return None

def client_program():
    print(time.time())
    server_address = ('192.168.1.102', 11234)  # The same IP and port as in your server script

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        # Connect to server
        sock.connect(server_address)
        print(f"Connected to {server_address[0]}:{server_address[1]}")

        while True:
            # Receive the data
            data = sock.recv(1024).decode('utf-8')
            if data:
                # Process the received pose data
                pose = parse_pose(data.strip())
                if pose:
                    print(f"[{time.time()}]Received Pose:")
                    print(f"  Pos(xyz):  {pose['position']['x']}, {pose['position']['y']}, {pose['position']['z']}")
                    print(f"  Ori(xyzw): {pose['orientation']['x']}, {pose['orientation']['y']}, {pose['orientation']['z']}, {pose['orientation']['w']}")
            else:
                break

    except Exception as e:
        print(f"Error: {e}")

    finally:
        sock.close()

if __name__ == '__main__':
    client_program()
