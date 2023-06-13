import serial
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time



# Conecta con el vehiculo (drone)
#vehicle = connect("tcp:172.20.10.2:5762", wait_ready=True)
vehicle = connect('/dev/ttyAMA1' , baud = 57600,wait_ready=True)
# Configuracion del puerto serial
port = '/dev/ttyUSB0'  
baud_rate = 57600  

# Inicializar el objeto Serial
ser = serial.Serial(port, baud_rate)

print('//////////////Programa Iniciado////////////////////// \n')

# Arma el vehiculo
vehicle.mode = VehicleMode("STABILIZE")
#vehicle.armed = True
takeoff = False

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        
    
    vehicle.send_mavlink(msg)
    
    



# Leer datos del puerto serial y mostrarlos por pantalla
while True:
    try:
        # Leer una linea de datos del puerto serial
        line = ser.read().decode('utf-8')
        # Mostrar la linea de datos por pantalla
        print(line)
        while takeoff != True:
            line = ser.read().decode('utf-8')
            if line == "6":
                vehicle.armed = True
                print("----------------ARMED-----------------")
                time.sleep(3)
                vehicle.simple_takeoff(5)
                takeoff = True
                print("----------------DESPEGANDO-----------------")
                time.sleep(5)

        if takeoff == True:
            #vehicle.armed = True
            if line == "1": #derecha
                SOUTH=1
                UP=0   
                DURATION = 0.1

                #Fly south and up.
                send_ned_velocity(SOUTH,0,0,DURATION)

            if line == "2": #izq
                SOUTH=-1
                UP=0   
                DURATION = 0.1

                send_ned_velocity(SOUTH,0,0,DURATION)

            if line == "3": #adelante
                EAST=1
                UP=0   
                DURATION = 0.1

                #Fly south and up.
                send_ned_velocity(0,EAST,0,DURATION)
                

            if line == "4": #atras
                EAST=-1
                UP=0   
                DURATION = 0.1

                #Fly south and up.
                send_ned_velocity(0,EAST,0,DURATION)
                
            if line == "9":
                UP=-0.2   
                DURATION = 0.1

                #Fly south and up.
                send_ned_velocity(0,0,UP,DURATION)

            if line == "8":
                UP=0.2   
                DURATION = 0.1

                #Fly south and up.
                send_ned_velocity(0,0,UP,DURATION)
           

    except:
        print('ERROR EN EL CODIGO')
        # Detener el bucle si se presiona Ctrl + C
        # Aterriza el vehiculo
        vehicle.mode = VehicleMode("LAND")
        break

# Desarma el vehiculo
vehicle.armed = False
print("----------------FINISHED-----------------")
# Cierra la conexion con el vehiculo
vehicle.close()
# Cerrar la conexion del puerto serial
ser.close()


