import argparse
import can
import subprocess
import serial.tools.list_ports
import os
from dotenv import load_dotenv
import curses

# Load environment variables from a .env file
# load_dotenv()

def change_permissions(file_path, permissions, password):
    try:
        if not file_path:
            raise ValueError("Invalid file path provided.")
        
        command = ["sudo", "-S", "chmod", permissions, file_path]
        result = subprocess.run(
            command,
            input=f"{password}\n",
            text=True,
            capture_output=True,
            check=True
        )
        return f"Permissions changed for {file_path}. Output: {result.stdout.strip()}"
    except subprocess.CalledProcessError as e:
        return f"Error changing permissions: {e}"
    except Exception as e:
        return f"Unexpected error: {e}"

def find_can_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "CAN" in port.description or "USB2CAN" in port.description:
            return port.device
        print(ports)
    return None

def crc(data, can_id):
    return sum(data + [can_id]) & 0xFF

def send_message(bus, msg, log_func):
    try:
        bus.send(msg)
        data_bytes = ', '.join([f'0x{byte:02X}' for byte in msg.data])
        log_func(f"Sent message: can.Message(arbitration_id=0x{msg.arbitration_id:02X}, data=[{data_bytes}], is_extended_id=False)")
    except Exception as e:
        log_func(f"Failed to send message: {e}")

def read(bus, can_id, log_func):
    data = [0x31]
    data.append(crc(data, can_id))

    try:
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        send_message(bus, msg, log_func)
    except Exception as e:
        log_func(f"Failed to read: {e}")

def notify_on_receive(msg, log_func):
    if msg.data[0] == 0x31:
        data_bytes = '0x' + ''.join([f'{byte:02X}' for byte in msg.data[1:7]])
        out = f"Angle (addition): {data_bytes}"
        log_func(out)
        return
    if msg.data[0] == 0x30:
        data_bytes = ', '.join([f'0x{byte:02X}' for byte in msg.data[1:7]])
        out = f"Angle (carry): {data_bytes}"
        log_func(out)
        return
    else:
        data_bytes = ', '.join([f'0x{byte:02X}' for byte in msg.data[:len(msg.data)]])
        out = f"Received: can.Message(arbitration_id=0x{msg.arbitration_id:02X}, data=[{data_bytes}], is_extended_id=False)"
        log_func(out)
        return

def move(bus, can_id, position, log_func, speed, acceleration=2):
    if position < 0:
        position += (1 << 24)
    data = [
        0xF4,
        (speed >> 8) & 0xFF,
        speed & 0xFF,
        acceleration,
        (position >> 16) & 0xFF,
        (position >> 8) & 0xFF,
        position & 0xFF
    ]
    data.append(crc(data, can_id))

    try:
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        send_message(bus, msg, log_func)
    except Exception as e:
        log_func(f"Failed to move: {e}")

def set_mstep(bus, can_id, mstep, log_func):
    data = [0x84, mstep]

    data.append(crc(data, can_id))

    try:
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        send_message(bus, msg, log_func)
    except Exception as e:
        log_func(f"Failed to move: {e}")


def set_oled_off(bus, can_id, off, log_func):
    data = [0x87, off]

    data.append(crc(data, can_id))

    try:
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        send_message(bus, msg, log_func)
    except Exception as e:
        log_func(f"Failed to set oled: {e}")

def main(stdscr):

    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--position', type=str, default="0x4000")
    parser.add_argument('-i', '--id', type=str, default="0x01")
    parser.add_argument('-v', '--velocity', type=int, default=600)
    parser.add_argument('-m', '--mstep', type=int, default=2)
    parser.add_argument('-o', '--oled', type=int, default=0)


    
    args = parser.parse_args()

    # Setup curses
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)

    # Create a subwindow for logging
    log_window = stdscr.subwin(curses.LINES - 2, curses.COLS, 0, 0)
    log_window.scrollok(True)

    def log_func(message):
        log_window.addstr(message + '\n')
        log_window.refresh()

    # CAN setup
    # password = os.getenv('SUDO_PASSWORD')
    can_port = find_can_port()

    position = int(args.position, 16)
    can_id = int(args.id, 16)
    speed = args.velocity
    mstep = args.mstep
    oled = args.oled

    log_func(f'Position increment is set to {hex(position)}.')
    log_func(f'CAN ID is set to {hex(can_id)}.')
    log_func(f'Speed is set to {speed}.')
    log_func(f'MSTEP is set to {mstep}.')
    log_func(f'Auto-turnoff is set to {oled}.')


    # arduino_port = '/dev/ttyACM1'
    # change_permissions(arduino_port, "777", 'arianne')

    # baud_rate = 9600
    # ser = serial.Serial(arduino_port, baud_rate)

    if not can_port:
        log_func("No CAN port found. Exiting.")
        return

    change_permissions(can_port, "777", 'louis')

    try:
        bus = can.interface.Bus(can_port, 'slcan', bitrate=500000)
        notifier = can.Notifier(bus, [lambda msg: notify_on_receive(msg, log_func)])
        log_func("Initialized CAN.")

        # Main loop
        while True:
            key = stdscr.getch()
            if key == curses.KEY_UP:
                move(bus, can_id, position ,log_func, speed)
            elif key == curses.KEY_DOWN:
                move(bus, can_id, -(position), log_func, speed)
            elif key == ord('1'):
                move(bus, can_id, 0x100, log_func, speed)
            elif key == ord('2'):
                move(bus, can_id, 0x200, log_func, speed)
            
            elif key == ord('x'):
                # ser.write(str(105).encode())
                move(bus, 0x01, -0x6000, log_func, speed)
                set_mstep(bus, 0x01, 3, log_func)
                move(bus, 0x02, -0x30000, log_func, 800)
                set_mstep(bus, 0x02, 2, log_func)
                move(bus, 0x03, -0x18000, log_func, speed)
                set_mstep(bus, 0x03, 3, log_func)
                move(bus, 0x05, 0x3C000, log_func, speed)
                set_mstep(bus, 0x04, 10, log_func)
            elif key == ord('z'):
                # ser.write(str(70).encode())
                move(bus, 0x01, 0x6000, log_func, speed)
                move(bus, 0x02, 0x30000, log_func, speed)
                move(bus, 0x03, 0x18000, log_func, speed)
                move(bus, 0x05, -0x3C000, log_func, speed)

            elif key == ord('m'):
                set_mstep(bus, can_id, mstep, log_func)
            elif key == ord('y'):
                move(bus, 0x02, 0x2000, log_func, speed)
            elif key == ord('h'):
                move(bus, 0x02, -0x1000, log_func, speed)
            elif key == ord('r'):
                read(bus, can_id, log_func)
            # elif key == ord('o'):
            #     ser.write(str(105).encode())
            # elif key == ord('c'):
            #     ser.write(str(70).encode())
            elif key == ord('q'):
                break
    except KeyboardInterrupt:
        log_func("Interrupted by user.")
    except Exception as e:
        log_func(f"Error: {e}")
    finally:
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        if 'bus' in locals():
            bus.shutdown()
        log_func("Cleaned up resources.")

if __name__ == "__main__":
    curses.wrapper(main)
