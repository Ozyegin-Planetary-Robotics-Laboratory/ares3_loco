import os
import can
import time
import subprocess  
import math
import gps_lib_desktop
import getGps_desktop
import pyzed.sl as sl
import logging 

#mv_1 mv_2 ---> front_right
#mv_3 mv_4 ---> back_right

#logging.basicConfig(level=logging.DEBUG)



control_modes = {"SET_DUTY":0, "SET_CURRENT":1, "SET_CURRENT_BRAKE":2, "SET_RPM":3, "SET_POS":4, "SET_ORIGIN_HERE":5, "SET_POS_SPD":6}
can_channel = os.getenv("MOTOR_CAN")
tx_timeout =  0.1 #second
rx_timeout =  0.1 #second
can_bitrate = 500000
rpm_constant = 200

heading_offset = None
magnetic_declination = None

initial_run = 1

max_number_of_fail = 12

bus = None

motors_dictionary = {"mv_1":251, "mv_2":252, "mv_3":-1, "mv_4":254} #Put -1 if motor can not be reachable
motors_directiones = {"mv_1":1, "mv_2":-1, "mv_3":1, "mv_4":-1} #Put -1 for reverse

motor_situations = {value: [0, 0, 0, 0, 0, 0, 0, 0] for key,value in motors_dictionary.items()}

fail_counter = 0 

buffer_vertical = [0]
buffer_angular = [0]

buffer_length = 5


def start_bus():
    global bus
    subprocess.run(["sudo","ip","link","set",can_channel,"up","type","can","bitrate",str(can_bitrate)])
    bus = can.interface.Bus(channel=can_channel, bustype="socketcan", bitrate = can_bitrate) #Open the bus.

def can_transmit(arb_id,message_data): #arbitation_id(hex) first 21 bits are for select control mode, last 8 bits are for motor_id  &&&& message_data is a array consists of 8bit parts
    global bus
    global fail_counter

    message = can.Message(arbitration_id=arb_id, data = message_data, is_extended_id= True,)

    try:
        bus.send(message, timeout=tx_timeout)

    except can.CanError as e:
        print(f"Failed to send message! Error: {e}")
        fail_counter += 1

def pad_to_eight(input_list):
    return input_list + [0] * (8 - len(input_list))

def can_recive():
        global motor_situations
        global fail_counter
        try:
            message = bus.recv(timeout=rx_timeout)
            if message is not None:
                recived_id = int(bin(int(message.arbitration_id))[2:].zfill(29)[-8:],2)
                recived_situation = bin(int(message.arbitration_id))[2:].zfill(29)[:-8]

                motor_situations[recived_id] = pad_to_eight([message.data[0], message.data[1], message.data[2], message.data[3], message.data[4], message.data[5], message.data[6], message.data[7]])
                
                if len(message.data) == 8:
                    high_byte_of_position = message.data[0]
                    low_byte_of_position = message.data[1]
                    high_byte_of_speed = message.data[2]
                    low_byte_of_speed = message.data[3]
                    high_byte_of_current = message.data[4]
                    low_byte_of_current = message.data[5]
                    motor_temperature = message.data[6] # In celcius degree
                    error_code = message.data[7] #Error code 


                    position_recived = int(bin(high_byte_of_position)[2:].zfill(8) + bin(low_byte_of_position)[2:].zfill(8),2) / 10.0 # Position in degrees
                    speed_recived = int(bin(high_byte_of_speed)[2:].zfill(8) + bin(low_byte_of_speed)[2:].zfill(8),2) * 10.0 # Speed in rpm
                    current_recived = int(bin(high_byte_of_current)[2:].zfill(8) + bin(low_byte_of_current)[2:].zfill(8),2) / 100.0 # Current in amps
                    
                    if error_code == 0:
                        return recived_id,position_recived,speed_recived,current_recived,motor_temperature #,recived_situation

                    else:
                        fail_counter += 1
                        print("Error in motor: " + str(recived_id) + " --- Error Code: " + str(error_code))
                        return error_code

                else:
                    fail_counter += 1
                    print("Incorrect number of data bytes recived.")
            
            else:
                print("No message recived!")
                fail_counter += 1
                return None
                 
        except can.CanError:
            print("Failed to recive message!")
            fail_counter += 1
    

def set_duty(motor_id, duty_cycle):
    if -1 <= duty_cycle <= 1: 

        duty = duty_cycle*100000
        duty = bin(int(duty))
        duty = duty[duty.index("b")+1:].zfill(32)

        if duty_cycle < 0:
             n = len(duty)
             flipped = ''.join('1' if bit == '0' else '0' for bit in duty)
             duty = bin(int(flipped, 2) + 1)[2:].zfill(n)
            
        duty_bytes = [int(duty[0:8],2), int(duty[8:16],2), int(duty[16:24],2), int(duty[24:32],2)]

        sitaution_id = bin(control_modes["SET_DUTY"])[2:].zfill(21)
        motor_id_ = bin(motor_id)[2:].zfill(8)
        arb_id = int(sitaution_id + motor_id_,2)
        
        can_transmit(arb_id,duty_bytes)
    else:
        print("Incorrect duty cycle!")
        return None


def set_current_loop(motor_id, float_current): #A
    if -60 <= float_current <= 60: 

        current = float_current*1000
        current = bin(int(current))
        current = current[current.index("b")+1:].zfill(32)
        if float_current < 0:
            n = len(current)
            flipped = ''.join('1' if bit == '0' else '0' for bit in current)
            current = bin(int(flipped, 2) + 1)[2:].zfill(n)
        curret_bytes = [int(current[0:8],2), int(current[8:16],2), int(current[16:24],2), int(current[24:32],2)]

        sitaution_id = bin(control_modes["SET_CURRENT"])[2:].zfill(21)
        motor_id_ = bin(motor_id)[2:].zfill(8)
        arb_id = int(sitaution_id + motor_id_,2)
        
        can_transmit(arb_id,curret_bytes)
    else:
        print("Incorrect float current!")
        return None

def set_current_brake(motor_id, brake_current): #A
    if 0 <= brake_current <= 60: 

        current = brake_current*1000
        current = bin(int(current))
        current = current[current.index("b")+1:].zfill(32)
        if brake_current < 0:
            n = len(current)
            flipped = ''.join('1' if bit == '0' else '0' for bit in current)
            current = bin(int(flipped, 2) + 1)[2:].zfill(n)
        curret_bytes = [int(current[0:8],2), int(current[8:16],2), int(current[16:24],2), int(current[24:32],2)]

        sitaution_id = bin(control_modes["SET_CURRENT_BRAKE"])[2:].zfill(21)
        motor_id_ = bin(motor_id)[2:].zfill(8)
        arb_id = int(sitaution_id + motor_id_,2)
        
        can_transmit(arb_id,curret_bytes)
    else:
        print("Incorrect brake current!")
        return None
    

def set_velocity_loop(motor_id, velocity,rpm_constant = rpm_constant): #RPM
    if -1e5/rpm_constant <= velocity <= 1e5/rpm_constant: 

        velocity_ = bin(int(velocity*rpm_constant))
        velocity_ = velocity_[velocity_.index("b")+1:].zfill(32)

        if velocity < 0:
            n = len(velocity_)
            flipped = ''.join('1' if bit == '0' else '0' for bit in velocity_)
            velocity_ = bin(int(flipped, 2) + 1)[2:].zfill(n)
        velocity_bytes = [int(velocity_[0:8],2), int(velocity_[8:16],2), int(velocity_[16:24],2), int(velocity_[24:32],2)]

        sitaution_id = bin(control_modes["SET_RPM"])[2:].zfill(21)
        motor_id_ = bin(motor_id)[2:].zfill(8)
        arb_id = int(sitaution_id + motor_id_,2)
        
        can_transmit(arb_id,velocity_bytes)
    else:
        print("Incorrect rpm!")
        return None

def set_position_loop(motor_id, position): #Degree
    if -36000 <= position <= 36000: 

        position_ = bin(int(position*1e4))
        position_ = position_[position_.index("b")+1:].zfill(32)

        if position < 0:
            n = len(position_)
            flipped = ''.join('1' if bit == '0' else '0' for bit in position_)
            position_ = bin(int(flipped, 2) + 1)[2:].zfill(n)
        position_bytes = [int(position_[0:8],2), int(position_[8:16],2), int(position_[16:24],2), int(position_[24:32],2)]

        sitaution_id = bin(control_modes["SET_POS"])[2:].zfill(21)
        motor_id_ = bin(motor_id)[2:].zfill(8)
        arb_id = int(sitaution_id + motor_id_,2)
        
        can_transmit(arb_id,position_bytes)
    else:
        print("Incorrect position!")
        return None


def set_origin_here(motor_id):
    set_position = [int(bin(1)[2:].zfill(32),2)]

    sitaution_id = bin(control_modes["SET_ORIGIN_HERE"])[2:].zfill(21)
    motor_id_ = bin(motor_id)[2:].zfill(8)
    arb_id = int(sitaution_id + motor_id_,2)
        
    can_transmit(arb_id,set_position)


def set_position_velocity_loop(motor_id, position, velocity, acceleration, rpm_constant = rpm_constant): #Degree, RPM , RPM/s
    if (-36000 <= position <= 36000) and (-327680/rpm_constant <= velocity <= 327680/rpm_constant) and (0 <= acceleration <= 327670): 

        position_ = bin(int(position*1e4))
        position_ = position_[position_.index("b")+1:].zfill(32)

        if position < 0:
            n = len(position_)
            flipped = ''.join('1' if bit == '0' else '0' for bit in position_)
            position_ = bin(int(flipped, 2) + 1)[2:].zfill(n)
     
        velocity_ = bin(int(velocity*rpm_constant/10))
        velocity_ = velocity_[velocity_.index("b")+1:].zfill(16)

        if velocity < 0:
            n = len(velocity_)
            flipped = ''.join('1' if bit == '0' else '0' for bit in velocity_)
            velocity_ = bin(int(flipped, 2) + 1)[2:].zfill(n)
        

        acceleration_ = bin(int(acceleration/10))
        acceleration_ = acceleration_[acceleration_.index("b")+1:].zfill(16)
        
        sitaution_id = bin(control_modes["SET_POS_SPD"])[2:].zfill(21)
        motor_id_ = bin(motor_id)[2:].zfill(8)
        arb_id = int(sitaution_id + motor_id_,2)

        send_bytes = [int(position_[0:8],2), int(position_[8:16],2), int(position_[16:24],2), int(position_[24:32],2),int(velocity_[0:8],2), int(position_[8:16],2),int(acceleration_[0:8],2), int(acceleration_[8:16],2)]


        can_transmit(arb_id,send_bytes)
    else:
        print("Incorrect position or velocity or acceleration!")
        return None


def stop_bus():
    bus.shutdown()
    subprocess.run(["sudo","ip","link","set",can_channel,"down"])




def linear_velocity_multiplier_function(y,dead_zone):
    b = 50
    if y<0:
        c = -b*math.e**dead_zone
        return -(b*math.e**-y + c)

    else:
        c = -b*math.e**dead_zone
        return b*math.e**y + c

def angular_velocity_multiplier_function(x,dead_zone):
    b = 50
    if x<0:
        c = -b*math.e**dead_zone
        return -(b*math.e**-x + c)

    else:
        c = -b*math.e**dead_zone
        return b*math.e**x + c

def sticks_2_velocities(x,y): #mv_1 --> front_left !!!!!!!!!  mv_2--> front_right !!!!!!! mv_3-->back_left !!!!!!!!!! mv_4----->back_right 
    
    x_dead_zone = 0.1
    y_dead_zone = 0.1
    diffrential_constant = 0.4
    maneuverability_constant = 0.4

    if (-x_dead_zone < x < x_dead_zone) and (-y_dead_zone < y < y_dead_zone):
        return [0,0,0,0]
    elif (-x_dead_zone < x < x_dead_zone):
        y_ = linear_velocity_multiplier_function(y,y_dead_zone)
        y_front = diffrential_constant*y_*2
        y_back = (1-diffrential_constant)*y_*2

        return [y_front,y_front,y_back,y_back]

    elif (-y_dead_zone < y < y_dead_zone):
        x_ = angular_velocity_multiplier_function(x,x_dead_zone)
        return [x_,-x_,x_,-x_]
    
    else:
        y_ = linear_velocity_multiplier_function(y,y_dead_zone)
        x_ = angular_velocity_multiplier_function(x,x_dead_zone)

        y_front = diffrential_constant*y_*2* (1-maneuverability_constant)
        y_back = (1-diffrential_constant)*y_*2 * (1-maneuverability_constant)
        x_front = diffrential_constant*x_*2 * maneuverability_constant
        x_back = (1-diffrential_constant)*x_*2 *maneuverability_constant

        return [y_front+x_front,y_front-x_front,y_back+x_back,y_back-x_back]


def velocity_control_loco(x,y):
    global motor_situations
    global motors_dictionary #motors ids
    global fail_counter
    global max_number_of_fail
    global buffer_length,buffer_angular,buffer_vertical

    if fail_counter > max_number_of_fail:
        motor_situations = {value: [0, 0, 0, 0, 0, 0, 0, 0] for key,value in motors_dictionary.items()}
        fail_counter = 0
        print("Bus is reseting!")
        stop_bus()
        time.sleep(1)
        start_bus()
        time.sleep(1)

    buffer_vertical.append(y)
    buffer_angular.append(x)

    if len(buffer_vertical) > buffer_length:
        buffer_vertical.pop(0)
    
    if len(buffer_angular) > buffer_length:
        buffer_angular.pop(0)

    y = sum(buffer_vertical)/len(buffer_vertical)
    x = sum(buffer_angular)/len(buffer_angular)

    velocities = sticks_2_velocities(x,y)
  
    for i in range(4):
        id_ = motors_dictionary["mv_"+str(i+1)]
        motors_directione = motors_directiones["mv_"+str(i+1)]
        if id_ != -1:
            set_velocity_loop(id_,velocities[i]*motors_directione)
    
    print(f"Fail count: {fail_counter}")


def shortest_angle_difference(angle1, angle2):
    diff = (angle2 - angle1 + 180) % 360 - 180
    return diff
 
def set_angle(target_angle, zed, sensors_data): 
    global initial_run, heading_offset, magnetic_declination
    if initial_run and zed is None:
        print('Initial run')
        init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                                    coordinate_units=sl.UNIT.METER,
                                    coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
        zed = sl.Camera()
        sensors_data = sl.SensorsData()
        status = zed.open(init)
        initial_run = 0
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            
    
    magnetic_heading = target_angle + 180
    while zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE) #  Get frame synchronized sensor data
        magnetometer_data = sensors_data.get_magnetometer_data()
        magnetic_heading = magnetometer_data.magnetic_heading 
        magnetic_heading = (magnetic_heading + heading_offset) % 360
        heading = magnetic_heading + magnetic_declination

        #print(magnetic_heading,target_angle)
        for _ in range(5):
            velocity_control_loco(math.copysign(0.2,shortest_angle_difference(heading,target_angle)),0)
            time.sleep(0.1)
        if abs(target_angle-heading)<5:
            break
        time.sleep(2) 

    if initial_run == 0:
        zed.close()   

def goto_coordinates(target_coordinates,zed = None, sensors_data = None, timeout = 1, heading_offset_init = 0, magnetic_declination_init = 0):
    global heading_offset,magnetic_declination

    heading_offset = heading_offset_init
    magnetic_declination = magnetic_declination_init

    coords = getGps_desktop.get_gps() #tuple(gps_lib_desktop.read_gps().values()) # pseudo coordinates getGps_desktop.get_gps()
    angle = gps_lib_desktop.calculate_vector(coords[0],coords[1],target_coordinates[0],target_coordinates[1])[1]
    print("Target Distance:",gps_lib_desktop.ll2meters(coords[0],coords[1],target_coordinates[0],target_coordinates[1]))
    print("Target Angle: ",angle)
    set_angle(angle,zed, sensors_data)
    time_start = time.time()

    while True:
        velocity_control_loco(0.0,0.4)

        if gps_lib_desktop.ll2meters(coords[0],coords[1],target_coordinates[0],target_coordinates[1]) < 2:
            return 1

        elif time.time() - time_start > timeout:
            print("Timeout")
            return -1
               


if __name__=="__main__":
    start_bus()
    #buffer_length = 1
    while True:
        #def set_position_velocity_loop(motor_id, position, velocity, acceleration, rpm_constant = rpm_constant): #Degree, RPM , RPM/s
        velocity_control_loco(0.0,0.8) #
        #set_position_velocity_loop(255, 7200, 1000, 2000, 200)
        #can_recive()
    stop_bus()
