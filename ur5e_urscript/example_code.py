import socket, yaml

'''
Using python script as: URScript generator + Network pip

We do not run URScript under python's interpreter, but we compose
URScript text in python and configure proper TCP socket for streaming that
to robot's controller.
'''

with open("../config/config.yml", "r") as f:
    cfg = yaml.safe_load(f)

ROBOT_IP = cfg['UR5E']['ROBOT_IP']
PRIMARY_PORT = cfg['UR5E']['PRIMARY_PORT']
SECONDARY_PORT = cfg['UR5E']['SECONDARY_PORT']
REALTIME_PORT = cfg['UR5E']['REALTIME_PORT']

# URScript command being sent to the robot
urscript_command_1 = "set_digital_out(1, False)"
# urscript_command_2 = "movej([1.57, -1.57, 1.57, -1.57, 1.57, 0], a=0.4, v=0.5)"


new_line = "\n"

def send_urscript_command(command: str):
    """
    This function takes the URScript command defined above, 
    connects to the robot server, and sends 
    the command to the specified port to be executed by the robot.
    """
    try:
        # Create a socket connection with the robot IP and port number defined above
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ROBOT_IP, PRIMARY_PORT))

        # Appends new line to the URScript command (the command will not execute without this)
        command = command+new_line
        
        # Send the command
        s.sendall(command.encode('utf-8'))
        
        # Close the connection
        s.close()

    except Exception as e:
        print(f"An error occurred: {e}")

send_urscript_command(urscript_command_1)
