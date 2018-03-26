import socket
from threading import Thread

host = ''  # ip of raspberry pi
port = 5560

IS_START = False
IS_STOP = False

results_path = "C:\\Users\\t8358568\\Desktop\\wifi\\results"


def setup_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Socket created.")
    try:
        sock.bind((host, port))
    except socket.error as msg:
        print(msg)
    print("Socket bind complete.")
    return sock


def setup_connection(s):
    s.listen(1)  # Allows one connection at a time.
    conn, address = s.accept()
    print("Connected to: " + address[0] + ":" + str(address[1]))
    return conn


def data_transfer(s, conn):
    # A big loop that sends/receives data until told not to.
    while True:
        # Receive the data
        data = conn.recv(1024)  # receive the data
        data = data.decode('utf-8')
        # Split the data such that you separate the command
        # from the rest of the data.
        data_message = data.split(' ', 1)
        command = data_message[0]
        if command == 'START':
            print("Starting")
            global IS_START
            IS_START = True
        elif command == 'END':
            print("Ending")
            global IS_STOP
            IS_STOP = True
        elif command == 'GET':
            print("Getting")
            send_results()
            s.close()
            break
        else:
            print("Unknown command")
        print("reply sent")
    conn.close()


def send_results():
    with open(results_path, 'rb') as results:
        while True:
            chunk = results.read(1024)
            print(chunk.decode('utf-8'))
            conn.send(chunk)
            if not chunk:
                break
        conn.send()
        results.close()
        print("Done sending")


def do_stuff():
    print("Do_stuff")
    while not IS_START:
        pass
    i = 0
    while not IS_STOP:
        # print(i)
        if i == 10000:
            conn.send("INTERRUPT".encode())
        i += 1
        # run the program
    print("done do_stuff")


s = setup_server()
conn = setup_connection(s)
t = Thread(target=data_transfer, args=(s, conn, ))
t.start()
do_stuff()