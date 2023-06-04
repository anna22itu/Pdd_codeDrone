import serial
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Conecta con el vehículo (drone)
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Configuración del puerto serial
port = 'COM5'  # Cambia el número del puerto según tu configuración
baud_rate = 57600  # La velocidad de transmisión debe coincidir con la configuración del dispositivo

# Inicializar el objeto Serial
ser = serial.Serial(port, baud_rate)

"""
0 --> parado
1 --> derecha
2 --> izquierda
5 --> hacia delante
4 --> hacia atrás
8 --> yaw 
10 -- land
"""

pitch_desired = 0.1  
roll_desired = 0.1
yaw_desired = 0.1

while not vehicle.is_armable:
    print('Esperando a que el vehículo sea armable...')
    time.sleep(1)


# Arma el vehículo
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

# Leer datos del puerto serial y mostrarlos por pantalla
while True:
    try:
        # Leer una línea de datos del puerto serial
        line = ser.read().decode('ascii')

        # Mostrar la línea de datos por pantalla
        print(line)

        if line == "0":
            # Envía el comando de control de vuelo al vehículo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch,
                vehicle.attitude.roll
            )

        if line ==  "1":
            # Envía el comando de control de vuelo al vehículo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch + pitch_desired,
                vehicle.attitude.roll
            )

        if line ==  "2":
            # Envía el comando de control de vuelo al vehículo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch + pitch_desired,
                vehicle.attitude.roll
            )

        if line ==  "4":
            # Envía el comando de control de vuelo al vehículo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch + pitch_desired,
                vehicle.attitude.roll
            )

        if line ==  "5":
            # Envía el comando de control de vuelo al vehículo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch + pitch_desired,
                vehicle.attitude.roll
            )

        if line == "6":
            # Envía el comando de control de vuelo al vehículo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch + pitch_desired,
                vehicle.attitude.roll
            )

        if line == "8":
            # Envía el comando de control de vuelo al vehículo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch + pitch_desired,
                vehicle.attitude.roll
            )

        if line == "10":
            vehicle.mode = VehicleMode("LAND")

    except KeyboardInterrupt:
        # Detener el bucle si se presiona Ctrl + C
        # Aterriza el vehículo
        vehicle.mode = VehicleMode("LAND")
        break


# Desarma el vehículo
vehicle.armed = False

# Cierra la conexión con el vehículo
vehicle.close()
# Cerrar la conexión del puerto serial
ser.close()


