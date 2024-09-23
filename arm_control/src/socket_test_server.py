import socket

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

def start_server():
    server_address = ('localhost', 10000)  # The same IP and port as in your main script

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the port
    sock.bind(server_address)

    # Listen for incoming connections
    sock.listen(1)
    print(f"Listening on {server_address[0]}:{server_address[1]}...")

    while True:
        # Wait for a connection
        print("Waiting for a connection...")
        connection, client_address = sock.accept()

        try:
            print(f"Connection from {client_address}")

            # Receive the data in small chunks and print it out
            while True:
                data = connection.recv(1024).decode('utf-8')
                if data:
                    # Process the received pose data
                    pose = parse_pose(data.strip())
                    if pose:
                        print("Received Pose:")
                        print(f"  Position: x={pose['position']['x']}, y={pose['position']['y']}, z={pose['position']['z']}")
                        print(f"  Orientation: x={pose['orientation']['x']}, y={pose['orientation']['y']}, z={pose['orientation']['z']}, w={pose['orientation']['w']}")
                else:
                    break

        except Exception as e:
            print(f"Error during connection handling: {e}")

        finally:
            # Clean up the connection
            connection.close()

if __name__ == '__main__':
    start_server()
