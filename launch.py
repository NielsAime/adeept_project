"""
Script de lancement global — équivalent d'un roslaunch.

Lance automatiquement :
  1. main_robot.py sur le Raspberry Pi via SSH
  2. main_pc.py en local (PC)

Usage :
  python launch.py

Dépendances :
  pip install paramiko
"""

import subprocess
import sys
import time
import socket
import threading
import paramiko

# ---------------------------------------------------------------------------
# Configuration — à adapter
# ---------------------------------------------------------------------------
ROBOT_USER  = 'adeept14'
ROBOT_IP    = '192.168.X.X'   # <-- remplace par l'IP du robot sur ton réseau
ROBOT_PORT  = 5000
SSH_PORT    = 22

ROBOT_SCRIPT = '~/robot_workspace/main_robot.py'

# ---------------------------------------------------------------------------

def wait_for_tcp(host, port, timeout=30):
    """Attend que le serveur TCP du robot soit prêt."""
    print(f"[LAUNCH] Attente du serveur TCP sur {host}:{port}...", flush=True)
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=1):
                print("[LAUNCH] Serveur TCP prêt.", flush=True)
                return True
        except (ConnectionRefusedError, OSError):
            time.sleep(0.5)
    print("[LAUNCH] ERREUR : serveur TCP non disponible après", timeout, "secondes.")
    return False


def stream_output(channel, prefix):
    """Affiche en temps réel la sortie SSH du robot."""
    for line in channel:
        print(f"[{prefix}] {line}", end='', flush=True)


def launch_robot_ssh():
    """Se connecte en SSH et lance main_robot.py sur le robot."""
    print(f"[LAUNCH] Connexion SSH à {ROBOT_USER}@{ROBOT_IP}...", flush=True)
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        ssh.connect(ROBOT_IP, port=SSH_PORT, username=ROBOT_USER)
    except Exception as e:
        print(f"[LAUNCH] Echec connexion SSH : {e}")
        sys.exit(1)

    print(f"[LAUNCH] Lancement de {ROBOT_SCRIPT} sur le robot...", flush=True)
    stdin, stdout, stderr = ssh.exec_command(f'python3 {ROBOT_SCRIPT}', get_pty=True)

    # Affichage des logs robot dans des threads séparés
    t_out = threading.Thread(target=stream_output, args=(stdout, 'ROBOT'), daemon=True)
    t_err = threading.Thread(target=stream_output, args=(stderr, 'ROBOT ERR'), daemon=True)
    t_out.start()
    t_err.start()

    return ssh  # garder la connexion ouverte


def launch_pc():
    """Lance main_pc.py en local."""
    print("[LAUNCH] Lancement de main_pc.py sur le PC...", flush=True)
    pc_script = 'pc_worspace_client/main_pc.py'
    proc = subprocess.Popen([sys.executable, pc_script])
    return proc


def main():
    # 1. Lancer le robot via SSH
    ssh = launch_robot_ssh()

    # 2. Attendre que le serveur TCP soit disponible
    if not wait_for_tcp(ROBOT_IP, ROBOT_PORT, timeout=30):
        ssh.close()
        sys.exit(1)

    # 3. Lancer le PC
    pc_proc = launch_pc()

    # 4. Attendre la fin du PC
    try:
        pc_proc.wait()
    except KeyboardInterrupt:
        print("\n[LAUNCH] Interruption clavier — arrêt.")
        pc_proc.terminate()

    # 5. Fermer SSH
    ssh.close()
    print("[LAUNCH] Terminé.")


if __name__ == '__main__':
    main()
