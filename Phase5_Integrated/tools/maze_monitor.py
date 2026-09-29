#!/usr/bin/env python3
"""
maze_monitor -- a PC terminal for the maze robot's Bluetooth link.

The phone apps show about twenty lines. A run produces a few thousand, and the
line that explains a crash is always the one that just scrolled off. This
keeps every line, on disk, with a timestamp, while you type commands.

    python3 maze_monitor.py                 # find the robot, connect, log
    python3 maze_monitor.py --port COM7
    python3 maze_monitor.py --mac AA:BB:CC:DD:EE:FF     (Linux, no pairing setup)
    python3 maze_monitor.py --list          # show candidate ports and exit

Everything received is written to logs/maze-YYYYmmdd-HHMMSS.log as it arrives
(flushed per line, so a reset or an unplug loses nothing). Anything you type
goes to the robot; lines starting with '/' are handled here instead:

    /q              quit
    /f <text>       only PRINT lines containing <text> -- the log keeps all
    /f              clear the filter
    /mark <note>    write a divider and note into the log
    /stats          lines, bytes, elapsed
    /log            path of the current log file

CONNECTING
  The ESP32 speaks Bluetooth Classic SPP (not BLE), so pair it first:

  Windows  Settings > Bluetooth > Add device > 'MazeSolver_P5'. Pairing creates two
           COM ports; use the OUTGOING one. This script will list them.
  Linux    bluetoothctl -> scan on / pair <MAC> / trust <MAC> / scan off,
           then pass --mac (a direct RFCOMM socket, nothing else needed).
           The channel is found over SDP; --channel overrides it. Failing
           that: sudo rfcomm bind 0 <MAC>  ->  --port /dev/rfcomm0
           NOTE the ESP32 takes ONE client at a time: close the phone app.
  macOS    pair in System Settings; the port appears as /dev/cu.MazeSolver_P5-*

Needs pyserial for the serial path:  pip install pyserial
--mac on Linux needs nothing but the standard library.
"""

import argparse
import os
import queue
import socket
import sys
import threading
import time
from datetime import datetime

# The firmware advertises this name -- SerialBT.begin("MazeSolver_P5").
BT_NAME = "mazesolver"
DEFAULT_NAME_HINTS = (BT_NAME, "maze", "esp32", "bt", "rfcomm", "silab", "cp210", "ch340")


# --------------------------------------------------------------------------
# transports: both expose read() -> bytes and write(bytes)
# --------------------------------------------------------------------------
class SerialLink:
    """A paired SPP device exposed as a COM port / tty. Works everywhere."""

    def __init__(self, port, baud):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is not installed.  pip install pyserial")
        self.port = port
        # A short timeout keeps the reader responsive to shutdown instead of
        # blocking forever on a robot that has been switched off.
        self.s = serial.Serial(port, baud, timeout=0.2)

    def read(self):
        return self.s.read(4096)

    def write(self, data):
        self.s.write(data)

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass

    def __str__(self):
        return self.port


def sdp_channel(mac):
    """Ask the device which RFCOMM channel its serial port is on.

    Nothing guarantees channel 1. The ESP32 stack picks a free one when it
    registers the SPP record, so a firmware that also advertises another
    profile can land on 2 or 3 -- and hardcoding 1 then fails in a way that
    looks like the robot is switched off. sdptool ships with bluez, which is
    already installed if pairing worked. Returns None if it cannot tell.
    """
    import re
    import subprocess
    try:
        out = subprocess.run(["sdptool", "browse", "--uuid", "1101", mac],
                             capture_output=True, text=True, timeout=20).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    m = re.search(r"Channel:\s*(\d+)", out)
    return int(m.group(1)) if m else None


def _explain(mac, err):
    """Turn an errno into the thing that is actually wrong."""
    import errno as E
    n = getattr(err, "errno", None)
    if isinstance(err, socket.timeout) or n == E.ETIMEDOUT:
        return ("The robot never answered.\n"
                "  Most likely, in order:\n"
                "   1. SOMETHING ELSE IS ALREADY CONNECTED. The ESP32 accepts\n"
                "      exactly one SPP client -- disconnect the phone app.\n"
                "   2. The ESP32 is off, resetting, or browning out.\n"
                "   3. Out of range, or the adapter is still busy scanning:\n"
                "        bluetoothctl -- scan off\n"
                "  Check it is reachable at all:  sudo l2ping -c 3 %s" % mac)
    if n == E.ECONNREFUSED:
        return ("The robot is there but refused that channel -- almost always\n"
                "  the wrong RFCOMM channel. Find the real one:\n"
                "    sdptool browse --uuid 1101 %s\n"
                "  then pass it:  --channel <N>" % mac)
    if n in (E.EBUSY, E.EADDRINUSE):
        return ("The channel is in use. Something else has the robot -- the\n"
                "  phone app, a second copy of this monitor, or a stale\n"
                "  binding:  sudo rfcomm release all")
    if n in (E.EHOSTDOWN, E.EHOSTUNREACH, E.ENETDOWN, E.ENETUNREACH):
        return ("The adapter or the link is down:\n"
                "    sudo systemctl start bluetooth && bluetoothctl -- power on")
    if n in (E.EPERM, E.EACCES):
        return ("Permission denied. Your user is probably not in the bluetooth\n"
                "  group:  sudo usermod -aG bluetooth $USER   (then log out/in)")
    return "Unexpected error: %r" % (err,)


class RfcommLink:
    """Linux only: talk RFCOMM straight to the MAC, skipping rfcomm bind."""

    def __init__(self, mac, channel=None, timeout=8.0, verbose=True):
        if not hasattr(socket, "AF_BLUETOOTH"):
            sys.exit("--mac needs Linux (AF_BLUETOOTH). Use --port instead.")
        self.mac = mac

        if channel:
            candidates = [channel]
        else:
            found = sdp_channel(mac)
            if found:
                if verbose:
                    print("SDP says the serial port is on channel %d" % found)
                candidates = [found]
            else:
                # SDP told us nothing (sdptool missing, or the device would not
                # answer a browse). Try the channels an ESP32 actually uses
                # rather than giving up on 1 alone.
                if verbose:
                    print("SDP gave no channel -- trying the usual ones")
                candidates = [1, 2, 3]

        last = None
        for ch in candidates:
            s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                              socket.BTPROTO_RFCOMM)
            s.settimeout(timeout)
            try:
                s.connect((mac, ch))
            except OSError as e:
                s.close()
                last = e
                if verbose and len(candidates) > 1:
                    print("  channel %d: %s" % (ch, e))
                continue
            self.channel = ch
            self.s = s
            self.s.settimeout(0.2)
            return

        sys.exit("\nCould not open %s.\n\n  %s\n" % (mac, _explain(mac, last)))

    def read(self):
        try:
            return self.s.recv(4096)
        except socket.timeout:
            return b""

    def write(self, data):
        self.s.sendall(data)

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass

    def __str__(self):
        return self.mac


def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        sys.exit("pyserial is not installed.  pip install pyserial")
    return list(lp.comports())


def pick_port():
    """Guess which port is the robot, and say why, rather than picking blind."""
    ports = list_ports()
    if not ports:
        sys.exit("No serial ports found. Pair the robot first -- see --help.")

    def score(p):
        blob = " ".join(str(x or "") for x in (p.device, p.description,
                                               p.manufacturer, p.name)).lower()
        return sum(3 if h == BT_NAME else 1
                   for h in DEFAULT_NAME_HINTS if h in blob)

    ranked = sorted(ports, key=score, reverse=True)
    best = ranked[0]
    if score(best) == 0:
        print("Could not tell which port is the robot. Candidates:")
        for p in ports:
            print("   %-20s %s" % (p.device, p.description))
        sys.exit("Re-run with --port <one of the above>.")
    if len(ranked) > 1 and score(ranked[1]) == score(best):
        print("More than one port looks plausible:")
        for p in ranked:
            if score(p) == score(best):
                print("   %-20s %s" % (p.device, p.description))
        print("Trying %s -- pass --port to choose another." % best.device)
    return best.device


# --------------------------------------------------------------------------
# the monitor
# --------------------------------------------------------------------------
class Monitor:
    def __init__(self, link, log_path, echo=True):
        self.link = link
        self.log = open(log_path, "a", buffering=1, encoding="utf-8",
                        errors="replace")
        self.log_path = log_path
        self.echo = echo
        self.filter = None
        self.stop = threading.Event()
        self.lines = 0
        self.bytes = 0
        self.t0 = time.time()
        self.out = queue.Queue()

    def _stamp(self):
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _emit(self, text, to_log=True):
        line = "%s  %s" % (self._stamp(), text)
        if to_log:
            self.log.write(line + "\n")
        if self.filter is None or self.filter in text.lower():
            sys.stdout.write(line + "\n")
            sys.stdout.flush()

    def reader(self):
        """Reassemble lines across packet boundaries -- SPP splits anywhere."""
        buf = b""
        while not self.stop.is_set():
            try:
                chunk = self.link.read()
            except Exception as e:
                self._emit("[link error: %s]" % e)
                self.stop.set()
                return
            if not chunk:
                continue
            self.bytes += len(chunk)
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("utf-8", "replace").rstrip("\r")
                self.lines += 1
                self._emit(text)

    def writer(self):
        while not self.stop.is_set():
            try:
                data = self.out.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.link.write(data)
            except Exception as e:
                self._emit("[send failed: %s]" % e)
                self.stop.set()

    def send(self, text):
        self.out.put((text + "\n").encode())
        if self.echo:
            self._emit(">>> " + text)

    def command(self, line):
        """Local commands. Returns False to quit."""
        parts = line[1:].split(None, 1)
        cmd = parts[0].lower() if parts else ""
        arg = parts[1] if len(parts) > 1 else ""
        if cmd in ("q", "quit", "exit"):
            return False
        elif cmd == "f":
            self.filter = arg.lower() or None
            self._emit("[filter: %s]" % (self.filter or "off"), to_log=False)
        elif cmd == "mark":
            self._emit("-" * 60, to_log=True)
            self._emit("MARK: " + arg)
            self._emit("-" * 60, to_log=True)
        elif cmd == "stats":
            self._emit("[%d lines, %d bytes, %.0f s, log %s]"
                       % (self.lines, self.bytes, time.time() - self.t0,
                          self.log_path), to_log=False)
        elif cmd == "log":
            self._emit("[%s]" % os.path.abspath(self.log_path), to_log=False)
        else:
            self._emit("[unknown local command /%s -- /q /f /mark /stats /log]"
                       % cmd, to_log=False)
        return True

    def run(self):
        threading.Thread(target=self.reader, daemon=True).start()
        threading.Thread(target=self.writer, daemon=True).start()
        self._emit("[connected to %s -- logging to %s]"
                   % (self.link, self.log_path))
        self._emit("[type MENU for the robot's menu, /q to quit]", to_log=False)
        try:
            for line in sys.stdin:
                if self.stop.is_set():
                    break
                line = line.rstrip("\n")
                if line.startswith("/"):
                    if not self.command(line):
                        break
                elif line:
                    self.send(line)
        except KeyboardInterrupt:
            pass
        finally:
            # A monitor that quits without stopping the robot leaves it
            # driving at whatever it was doing. Ask it to stop on the way out.
            try:
                self.link.write(b"STOP\n")
                time.sleep(0.2)
            except Exception:
                pass
            self.stop.set()
            self._emit("[%d lines over %.0f s -> %s]"
                       % (self.lines, time.time() - self.t0,
                          os.path.abspath(self.log_path)), to_log=False)
            self.link.close()
            self.log.close()


def main():
    ap = argparse.ArgumentParser(
        description="PC Bluetooth monitor for the maze robot.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--port", help="serial port (COM7, /dev/rfcomm0, ...)")
    ap.add_argument("--mac", help="Bluetooth MAC, Linux direct RFCOMM")
    ap.add_argument("--channel", type=int, default=0,
                    help="RFCOMM channel (default: ask the device over SDP)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="ignored over real SPP, but some stacks want it")
    ap.add_argument("--log", help="log file path (default logs/maze-<time>.log)")
    ap.add_argument("--list", action="store_true", help="list ports and exit")
    ap.add_argument("--no-echo", action="store_true",
                    help="do not show what you typed")
    a = ap.parse_args()

    if a.list:
        for p in list_ports():
            print("%-20s %s" % (p.device, p.description))
        return

    if a.log:
        path = a.log
    else:
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(d, exist_ok=True)
        path = os.path.join(
            d, "maze-%s.log" % datetime.now().strftime("%Y%m%d-%H%M%S"))

    if a.mac:
        link = RfcommLink(a.mac, a.channel or None)
    else:
        link = SerialLink(a.port or pick_port(), a.baud)

    Monitor(link, path, echo=not a.no_echo).run()


if __name__ == "__main__":
    main()
