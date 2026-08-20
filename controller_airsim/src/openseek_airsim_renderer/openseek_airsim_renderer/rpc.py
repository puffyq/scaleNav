import socket
import threading

import msgpack


class RpcError(RuntimeError):
    pass


class MessagePackRpcClient:
    """Minimal synchronous MessagePack-RPC client for the AirSim API."""

    def __init__(self, host, port, timeout):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._socket = None
        self._unpacker = None
        self._message_id = 0
        self._lock = threading.Lock()

    def close(self):
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None
                self._unpacker = None

    def call(self, method, *parameters):
        with self._lock:
            try:
                self._connect()
                message_id = self._message_id
                self._message_id += 1
                request = [0, message_id, method, list(parameters)]
                self._socket.sendall(msgpack.packb(request, use_bin_type=True))
                return self._receive(message_id)
            except Exception:
                self.close()
                raise

    def _connect(self):
        if self._socket is not None:
            return
        self._socket = socket.create_connection(
            (self.host, self.port), timeout=self.timeout
        )
        self._socket.settimeout(self.timeout)
        self._unpacker = msgpack.Unpacker(raw=False, strict_map_key=False)

    def _receive(self, expected_message_id):
        while True:
            for response in self._unpacker:
                if not isinstance(response, (list, tuple)) or len(response) != 4:
                    raise RpcError("invalid MessagePack-RPC response")
                message_type, message_id, error, result = response
                if message_type != 1 or message_id != expected_message_id:
                    continue
                if error:
                    raise RpcError(str(error))
                return result

            data = self._socket.recv(1024 * 1024)
            if not data:
                raise ConnectionError("AirSim RPC connection closed")
            self._unpacker.feed(data)
