import subprocess
import re
import requests

def get_arp_table():
    try:
        output = subprocess.check_output(['arp', '-a']).decode()
        return output
    except Exception as e:
        print(f"Error running arp: {e}")
        return ""

def parse_arp_table(arp_output):
    # Match IP and MAC addresses from arp -a output
    pattern = re.compile(r'\((.*?)\) at ([\w:]+)')
    return pattern.findall(arp_output)

def find_esp32_devices() -> list:
    esp_devices = []
    arp_output = get_arp_table()
    devices = parse_arp_table(arp_output)

    print(f"{'IP Address':<15} {'MAC Address':<20} {'Is ESP32?':<10}")
    print("-" * 60)

    for ip, mac in devices:
        if mac.startswith("e4:b0:63"):
            esp_devices.append((ip, mac))
            print(f"{ip:<15} {mac:<20} {'YES':<10}")

    return esp_devices

def is_uwb_device(ip: str) -> bool:
    try:
        response = requests.get(f'http://{ip}', timeout=5)
        if response.status_code == 200:
            print(response.text)
            return response.text == "Athena UWB"
    except requests.exceptions.RequestException:
        pass
    return False

def get_state(ip: str) -> list | None:
    try:
        response = requests.get(f'http://{ip}/state', timeout=5)
        if response.status_code == 200:
            return response.text.split(',')
    except requests.exceptions.RequestException:
        pass
    return None

def set_id(ips: list[str]) -> bool:
    id = 0x6
    for ip in ips:
        try:
            # Send the ID in the body of a POST request
            response = requests.post(f'http://{ip}/set_id', data=str(id), timeout=5)
            if response.status_code == 200:
                id += 1
                print(response.text)  # Optional: Print the response for debugging
                return response.text.strip() == "ID set successfully"
        except requests.exceptions.RequestException as e:
            print(f"Error setting ID: {e}")
        return False
    
def get_id(ip: str) -> int | None:
    try:
        response = requests.get(f'http://{ip}/get_id', timeout=5)
        if response.status_code == 200:
            return int(response.text)
    except requests.exceptions.RequestException:
        pass
    return None

if __name__ == "__main__":
    esp_devices = find_esp32_devices()

    uwb_devices = []
    for ip, mac in esp_devices:
        if is_uwb_device(ip):
            uwb_devices.append((ip, mac))

    # set_id([ip for ip, mac in uwb_devices])

    print("\nUWB Devices:")
    print("-" * 12)
    for ip, mac in uwb_devices:
        print(f"{ip} {get_state(ip)}")
