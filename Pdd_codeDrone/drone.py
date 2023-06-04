import serial
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Conecta con el veh�culo (drone)
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Configuraci�n del puerto serial
port = 'COM5'  # Cambia el n�mero del puerto seg�n tu configuraci�n
baud_rate = 57600  # La velocidad de transmisi�n debe coincidir con la configuraci�n del dispositivo

# Inicializar el objeto Serial
ser = serial.Serial(port, baud_rate)

"""
0 --> parado
1 --> derecha
2 --> izquierda
5 --> hacia delante
4 --> hacia atr�s
8 --> yaw 
10 -- land
"""

pitch_desired = 0.1  
roll_desired = 0.1
yaw_desired = 0.1

while not vehicle.is_armable:
    print('Esperando a que el veh�culo sea armable...')
    time.sleep(1)


# Arma el veh�culo
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

# Leer datos del puerto serial y mostrarlos por pantalla
while True:
    try:
        # Leer una l�nea de datos del puerto serial
        line = ser.read().decode('ascii')

        # Mostrar la l�nea de datos por pantalla
        print(line)

        if line == "0":
            # Env�a el comando de control de vuelo al veh�culo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch,
                vehicle.attitude.roll
            )

        if line ==  "1":
            # Env�a el comando de control de vuelo al veh�culo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch + pitch_desired,
                vehicle.attitude.roll
            )

        if line ==  "2":
            # Env�a el comando de control de vuelo al veh�culo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch + pitch_desired,
                vehicle.attitude.roll
            )

        if line ==  "4":
            # Env�a el comando de control de vuelo al veh�culo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch + pitch_desired,
                vehicle.attitude.roll
            )

        if line ==  "5":
            # Env�a el comando de control de vuelo al veh�culo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch + pitch_desired,
                vehicle.attitude.roll
            )

        if line == "6":
            # Env�a el comando de control de vuelo al veh�culo
            vehicle.simple_goto(
                vehicle.location.global_relative_frame,
                vehicle.attitude.yaw,
                vehicle.attitude.pitch + pitch_desired,
                vehicle.attitude.roll
            )

        if line == "8":
            # Env�a el comando de control de vuelo al veh�culo
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
        # Aterriza el veh�culo
        vehicle.mode = VehicleMode("LAND")
        break


# Desarma el veh�culo
vehicle.armed = False

# Cierra la conexi�n con el veh�culo
vehicle.close()
# Cerrar la conexi�n del puerto serial
ser.close()


