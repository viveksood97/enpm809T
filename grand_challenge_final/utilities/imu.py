

class IMU:
    def get_current_angle(self, ser, count):
        if(ser.in_waiting > 0):
            count += 1
            line = ser.readline()
            
            if(count > 10):
                
                line = line.rstrip().lstrip()

                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                angle = float(line)
                
                
                if(angle >= 180):
                    current_angle = angle - 360
                else:
                    current_angle = angle
        
            return current_angle, count