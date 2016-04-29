import socket, select
import time
import picamera
from datetime import datetime

with picamera.PiCamera() as camera:
    try:
        logfile = open("/home/pi/robot_server/logs/picam_simple_log.txt", 'a+')
    except IOError:
        print "WARNING: Could not open log file!!"

    camera.resolution = (800, 600)
    # camera.resolution = (1920, 1080)
    camera.framerate = 24

    server_socket = socket.socket()
    server_socket.bind(('0.0.0.0', 8000))
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.listen(0)

    logfile.write("Started Picam streaming server, listening on port 8000, T=")
    logfile.write(str(datetime.now()))
    logfile.write('\n')
    logfile.close()

    while 1:
        # Accept a single connection and make a file-like object out of it
        connection_socket = server_socket.accept()
        logfile = open("/home/pi/robot_server/logs/picam_simple_log.txt", 'a+')
        logfile.write("Acquired client connection, T=")
        logfile.write(str(datetime.now()))
        logfile.write('\n')
        logfile.close()
        connection = connection_socket[0].makefile('wb')
        camera.start_recording(connection, format='h264')
        logfile = open("/home/pi/robot_server/logs/picam_simple_log.txt", 'a+')
        logfile.write("Started recording at T=")
        logfile.write(str(datetime.now()))
        logfile.write('\n')
        logfile.close()
        while 1:
            try:
                data = connection_socket[0].recv(1024)
            except socket.error:
                # print "ERROR while reading socket! Closing connection"
                logfile = open("/home/pi/robot_server/logs/picam_simple_log.txt", 'a+')
                logfile.write("ERROR while reading socket! Closing this connection, T=")
                logfile.write(str(datetime.now()))
                logfile.write('\n')
                logfile.close()
                camera.stop_recording()
                connection.close()
                break
            if not data:
                # print "Closing connection due to client disconnect"
                logfile = open("/home/pi/robot_server/logs/picam_simple_log.txt", 'a+')
                logfile.write("Closing connection due to client disconnect, T=")
                logfile.write(str(datetime.now()))
                logfile.write('\n')
                logfile.close()
                camera.stop_recording()
                connection.close()
                break
            try:
                camera.wait_recording()
            except socket.error:
                # print "Socket error, closing!!"
                logfile = open("/home/pi/robot_server/logs/picam_simple_log.txt", 'a+')
                logfile.write("Socket error, closing!! T=")
                logfile.write(str(datetime.now()))
                logfile.write('\n')
                logfile.close()
                connection.close()
                camera.stop_recording()