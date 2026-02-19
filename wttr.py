# ответ выглядит как то так "Patchy rain nearby +11°C ←4km/h 71% 🌑"
# проверил скрипт на macOS, работает и получает ответ как в предыдущей строке
import serial
import time
import sys
from datetime import datetime

# PORT = "/dev/cu.usbserial-1410"
PORT = "/dev/cu.usbserial-143110"
BAUD = 115200

SSID = "MEO-EDC8ED"
PASSWORD = "2668EB941B"

WTTR_HOST = "wttr.in"
WTTR_PATH = "/Peniche?format=%C+%t+%w+%h+%m+%o"
WTTR_PORT = 80

SER_TIMEOUT = 0.1
CMD_TIMEOUT = 10
RECV_TIMEOUT = 10

# ========= Send AT =========
def send_cmd(ser, cmd, timeout=CMD_TIMEOUT):
    ser.write((cmd + "\r\n").encode())
    ser.flush()
    start = time.time()
    lines = []
    while True:
        if time.time() - start > timeout:
            return lines
        data = ser.read(512)
        if not data:
            continue
        text = data.decode(errors="ignore")
        for line in text.replace("\r", "").split("\n"):
            if not line:
                continue
            lines.append(line)
            if line == "OK" or line == "ERROR":
                return lines

# ========= CIPSEND + СРАЗУ ДАННЫЕ =========
def cipsend_and_write(ser, payload):
    ser.write(f"AT+CIPSEND={len(payload)}\r\n".encode())
    ser.flush()
    start = time.time()
    while True:
        if time.time() - start > 3:
            return False
        data = ser.read(128)
        if not data:
            continue
        text = data.decode(errors="ignore")
        if b">" in data:
            break

    # ВАЖНО: отправка сразу!
    ser.write(payload.encode())
    ser.flush()
    return True

# ========= ПРИЁМ TCP =========
def recv_tcp_data(ser, timeout=RECV_TIMEOUT):
    start = time.time()
    raw = b""
    saw_closed = False
    while True:
        if time.time() - start > timeout:
            break
        data = ser.read(1024)
        if not data:
            time.sleep(0.05)
            continue
        raw += data
        text = data.decode(errors="ignore")
        p = 1 
        for line in text.replace("\r", "").split("\n"):
            if line.strip():
                if "°C" in line:
                    line = line.replace("CLOSED", "").strip()
                    print(f'{line}')
                p = p + 1
                if line == "CLOSED":
                    saw_closed = True

        if saw_closed:
            break
    return raw

# ========= ИЗВЛЕЧЕНИЕ ПОГОДЫ =========
def extract_weather(raw):
    try:
        body = raw.split(b"\r\n\r\n", 1)[1]
        lines = body.decode(errors="ignore").splitlines()
        for l in lines:
            l = l.strip()
            if l:
                return l
    except Exception:
        pass
    return None

# ========= MAIN =========
def main():
    try:
        ser = serial.Serial(
            PORT,
            BAUD,
            timeout=SER_TIMEOUT,
            write_timeout=2
        )
    except Exception as e:
        sys.exit(1)

    # База
    send_cmd(ser, "AT")
    send_cmd(ser, "ATE0")
    send_cmd(ser, "AT+CWMODE=1")

    # Wi-Fi
    send_cmd(ser, f'AT+CWJAP="{SSID}","{PASSWORD}"', timeout=60)

    # TCP
    send_cmd(ser, "AT+CIPSTA?")
    send_cmd(ser, f'AT+CIPSTART="TCP","{WTTR_HOST}",{WTTR_PORT}', timeout=65)

    # HTTP (curl-like)
    http_req = (
        f"GET {WTTR_PATH} HTTP/1.1\r\n"
        f"Host: {WTTR_HOST}\r\n"
        "User-Agent: curl/7.68.0\r\n"
        "Accept: */*\r\n"
        "Connection: close\r\n"
        "\r\n"
    )
    if not cipsend_and_write(ser, http_req):
        ser.close()
        return

    raw = recv_tcp_data(ser)
    # Закрывать соединение не нужно, если уже было CLOSED
    weather = extract_weather(raw)
    ser.close()
i = 1
if __name__ == "__main__":
    while i == 1:
        main()

