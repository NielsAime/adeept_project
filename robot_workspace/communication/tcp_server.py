"""
Serveur TCP du robot (tourne sur le Raspberry Pi).

Protocole : messages JSON précédés de 4 octets (big-endian) de longueur.
Le serveur accepte un client à la fois (le PC).

Usage :
    server = CommandServer(port=5000)
    server.on_command(my_callback)   # callback appelé à chaque commande
    server.start()                   # bloquant
"""

import socket
import json
import struct
import threading
from typing import Callable, Optional


class CommandServer:
    def __init__(self, port: int = 5000):
        self.port      = port
        self._callback: Optional[Callable] = None
        self._running  = False

    def on_command(self, callback: Callable) -> None:
        """Enregistre la fonction appelée à la réception de chaque commande."""
        self._callback = callback

    # ------------------------------------------------------------------
    # Lecture robuste
    # ------------------------------------------------------------------
    def _recv_exact(self, sock: socket.socket, n: int) -> Optional[bytes]:
        """Lit exactement n octets depuis le socket."""
        buf = b''
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    # ------------------------------------------------------------------
    # Gestion d'un client
    # ------------------------------------------------------------------
    def _handle_client(self, conn: socket.socket, addr) -> None:
        print(f"[Robot] PC connecte depuis {addr}")
        try:
            while self._running:
                # Lecture du header : 4 octets → longueur du message
                raw_len = self._recv_exact(conn, 4)
                if raw_len is None:
                    break
                msg_len = struct.unpack('>I', raw_len)[0]

                # Lecture du payload JSON
                raw_msg = self._recv_exact(conn, msg_len)
                if raw_msg is None:
                    break

                command = json.loads(raw_msg.decode('utf-8'))
                if self._callback:
                    self._callback(command)

        except Exception as e:
            print(f"[Robot] Erreur client : {e}")
        finally:
            conn.close()
            print("[Robot] PC deconnecte")

    # ------------------------------------------------------------------
    # Démarrage du serveur (bloquant)
    # ------------------------------------------------------------------
    def start(self) -> None:
        self._running = True
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', self.port))
        server_sock.listen(1)
        print(f"[Robot] Serveur TCP en ecoute sur le port {self.port}")

        try:
            while self._running:
                conn, addr = server_sock.accept()
                t = threading.Thread(
                    target=self._handle_client,
                    args=(conn, addr),
                    daemon=True
                )
                t.start()
        finally:
            server_sock.close()

    def stop(self) -> None:
        self._running = False
