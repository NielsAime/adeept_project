"""
Client TCP du PC vers le robot.

Protocole : chaque message est un JSON précédé de 4 octets (big-endian)
indiquant la longueur du payload.

Format des commandes JSON :
  {"type": "move",  "left": 50,  "right": 50}  → avance
  {"type": "stop"}                               → arrêt
  {"type": "grasp"}                              → séquence de saisie
"""

import socket
import json
import struct
from typing import Dict, Any


class RobotClient:
    def __init__(self, host: str, port: int = 5000):
        self.host = host
        self.port = port
        self._sock: socket.socket | None = None

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(5.0)
        self._sock.connect((self.host, self.port))
        self._sock.settimeout(None)
        print(f"[PC] Connecté au robot {self.host}:{self.port}")

    def send_command(self, command: Dict[str, Any]) -> None:
        """
        Envoie une commande JSON au robot.
        Format : [4 octets longueur big-endian][payload JSON UTF-8]
        """
        if self._sock is None:
            return
        payload = json.dumps(command).encode('utf-8')
        header  = struct.pack('>I', len(payload))
        try:
            self._sock.sendall(header + payload)
        except (BrokenPipeError, OSError) as e:
            print(f"[PC] Erreur envoi : {e}")
            self._sock = None

    def disconnect(self) -> None:
        if self._sock:
            try:
                self.send_command({"type": "stop"})
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        print("[PC] Déconnecté du robot")
