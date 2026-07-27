import serial

COM_PORT = 'COM7'
BAUD_RATE = 115200
MAX_LINES = 501    

try:

    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"Connected {COM_PORT}. Collecting data...")
    
    with open('ketqua.csv', 'w', encoding='utf-8') as file:
        count = 0
        while count < MAX_LINES:

            raw_data = ser.readline()
            try:
                line = raw_data.decode('utf-8').strip()
                if line:
                    file.write(line + '\n')
                    count += 1
                    print(f"Saved {count}/{MAX_LINES}", end='\r')
            except UnicodeDecodeError:
                continue
            
    print("\Completed! File 'dataset.csv' saved.")
    ser.close()

except Exception as e:
    print(f"Lỗi: {e}")