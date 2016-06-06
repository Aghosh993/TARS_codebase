all:
	gcc control_client.c `sdl-config --cflags --libs` -o control_client
	gcc remote_server_kill.c -o servkill
clean:
	rm control_client
	rm servkill
