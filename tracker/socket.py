import socket
 
s = socket.socket()         
 
s.bind(('0.0.0.0', 8090 ))
s.listen(0)                 
 
while True:
 
    client, addr = s.accept()
 
    while True:
        content = client.recv(64)
 
        if len(content) ==0:
           break
 
        else:
            print(content)
 
    #print("Closing connection")
    client.close()
