import serial
import time

# Configurar el puerto serie
ser = serial.Serial('COM3', 9600, timeout=1)  # Cambia 'COM3' al puerto correspondiente
time.sleep(2)  # Esperar a que la conexión se establezca

positions = []

def parse_color_data(data):
    try:
        parts = data.strip().split(' ')
        if parts[0] == "COLOR":
            r = int(parts[1])
            g = int(parts[2])
            b = int(parts[3])
            pos = int(parts[5])
            return r, g, b, pos
    except Exception as e:
        print(f"Error parsing data: {e}")
    return None

def determine_color(r, g, b):
    if r > g and r > b:
        return "Rojo"
    elif g > r and g > b:
        return "Verde"
    elif b > r and b > g:
        return "Negro"
    else:
        return "Indeterminado"

while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        print(f"Received: {line}")
        color_data = parse_color_data(line)
        if color_data:
            r, g, b, pos = color_data
            color = determine_color(r, g, b)
            print(f"Color detectado: {color} (Rojo: {r}, Verde: {g}, Azul: {b}, Posición: {pos})")
            
            if color == "Rojo":
                positions.append(pos)
                ser.write(b'T\n')  # Comando para mover a la posición detectada
                print("Victima grave encontrada en la posición", pos)
                ser.write(b'Victima grave encontrada\n')  # Enviar mensaje por serial
            elif color == "Verde":
                positions.append(pos)
                ser.write(b'T\n')  # Comando para mover a la posición detectada
                print("Victima leve encontrada en la posición", pos)
                ser.write(b'Victima leve encontrada\n')  # Enviar mensaje por serial
            elif color == "Negro":
                ser.write(b'F\n')  # Comando para seguir adelante
                print("Obstaculo encontrado en la posición", pos)
                ser.write(b'Obstaculo encontrado\n')  # Enviar mensaje por serial
            else:
                ser.write(b'S\n')  # Comando para detenerse
        else:
            ser.write(b'S\n')  # Comando para detenerse
    
    time.sleep(1)

ser.close()