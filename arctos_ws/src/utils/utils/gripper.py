import serial



open_limit = 110
close_limit = 55

def main():
    # Replace '/dev/ttyUSB0' with the correct port for your Arduino
    port = '/dev/ttyACM2'
    baud_rate = 9600

    try:
        ser = serial.Serial(port, baud_rate)
        print("Connected to Arduino.")

        while True:
            if open_limit is not None and close_limit is not None:
                
                if close_limit < open_limit:
                    angle = int(input(f"Enter angle ({close_limit} - {open_limit}): "))
                    if close_limit <= angle <= open_limit:
                        ser.write(str(angle).encode())
                    else:
                        print(f"Please provide an angle between {close_limit} and {open_limit}.")
                else:
                    angle = int(input(f"Enter angle ({open_limit} - {close_limit}): "))
                    if open_limit <= angle <= close_limit:
                        ser.write(str(angle).encode())
                    else:
                        print(f"Please provide an angle between {open_limit} and {close_limit}.")
            else:
              angle = int(input("Enter angle: "))
              ser.write(str(angle).encode())

    except serial.SerialException as e:
        print(f"Error connecting to Arduino: {e}")
    finally:
        if ser.is_open():
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()