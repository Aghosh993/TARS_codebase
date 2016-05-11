import socket

# Sort of ripped out of https://docs.python.org/2/howto/sockets.html

class simple_connector:
	def __init__(self, sock=None, default_msg_size=1024):
		if sock is None:
			self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		else:
			self.sock = sock

		self.default_msg_size = default_msg_size

	def create_connection(self, host, port):
		self.sock.connect((host, port))

	def send_buffer(self, buffer):
		bytes_sent = 0
		while bytes_sent < self.default_msg_size:
			sent_nbytes = self.sock.send(buffer[bytes_sent:])
			if sent_nbytes == 0:
				raise RuntimeError("Welp on TX!! No Socket connection!!")
			bytes_sent = bytes_sent + sent_nbytes

	def recv_buffer(self):
		message_portions = []
		bytes_received = 0
		while bytes_received < self.default_msg_size:
			message_portion = self.sock.recv(min(self.default_msg_size - bytes_received, 2048))
			if message_portion == '':
				raise RuntimeError("Welp on RX!! No Socket connection!!")
			message_portions.append(message_portion)
			bytes_received = bytes_received + len(message_portion)
		return ''.join(message_portions)

	def close_conn(self):
		self.sock.close()

def main():
	sc = simple_connector(default_msg_size=7)
	sc.create_connection('192.168.2.149', 8001)
	sc.send_buffer("rec0000")
	sc.close_conn()

if __name__ == '__main__':
	main()