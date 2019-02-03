"""
Autonomous drone Mission control.
AUTHOR: Bashir Hassan
"""
try:
    while True:
        # Don't let the user try to arm until autopilot is ready
        while not vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(1)

        string = raw_input ('Enter Command: ')
        word = string.split()
        word1 = word[0]
        
        if word1 == 'takeoff':
            arm_and_takeoff(vehicle, 10)
            break
            

        elif word1 == 'fly':
            # mapping for MANUAL MODE
            if word[1] == 'manual':
                print 'Flight control: MANUAL'
                manual_control(vehicle)
                break

            # mapping for AUTO MODE    
            elif word[1] == 'auto':
                print 'Flight control: AUTO'
                arm_and_takeoff(vehicle, 10)
                point1 = LocationGlobalRelative(55.870586,-4.287632, 25)
                go_to(vehicle, point1)
                break

            elif word[1] == 'square':
                print 'Flight control: square'
                square(vehicle)
                break

            elif word[1] == 'diamond':
                print 'Flight control: diamond'
                diamond(vehicle)
                break
            
            else
               arm_and_takeoff(vehicle, 10)


        elif word1 == 'show':
            shows_data(vehicle)
            print 'Drone data showed!'        
            break    

        elif word1 == 'arm':
            arm(vehicle)
            time.sleep(1)
            print 'Drone is armed'
            break        
                
        elif word1 == 'darm':
            darm(vehicle)
            time.sleep(1)
            print "Drone is disarmed and landed!"
            break

        elif word1 == 'land':
            print "\n\nLanding!\n\n"
            vehicle.mode = VehicleMode("LAND")
            time.sleep(10)
            print 'The drone has landed!'
            break

        elif word1 == 'return':
            print "\n\nReturning to Launch!\n\n"
            vehicle.mode = VehicleMode("RTL")
            time.sleep(15)
            print 'The drone has returned!'
            break
        
        
        else:
            print 'Wrong Input....'
        
        time.sleep(0.1)
            
except KeyboardInterrupt:
	pass

finally:
    roll.stop_servo(17)
    pitch.stop_servo(18)
    throttle.stop_servo(27)
    yaw.stop_servo(22)
    aux.stop_servo(23)
