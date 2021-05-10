# Script to read the IMU raw_data 
#-------------------------------------------

# All the imports 
import smbus
bus = smbus.SMBus(1) 
import numpy as np
import time 

# IMU class to init() and read the raw data bits 
# MPU9250 class 

class MPU9250:
    
    def __init__(self):
        
        # Class variables 
        # MPU9250 Registers
        self.MPU9250_ADDR = 0x68
        self.PWR_MGMT_1   = 0x6B
        self.SMPLRT_DIV   = 0x19
        self.CONFIG       = 0x1A
        self.INT_ENABLE   = 0x38
        self.TEMP_OUT_H   = 0x41

        # Gyro
        self.GYRO_CONFIG  = 0x1B
        self.GYRO_XOUT_H  = 0x43
        self.GYRO_YOUT_H  = 0x45
        self.GYRO_ZOUT_H  = 0x47

        #Accel
        self.ACCEL_CONFIG = 0x1C
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F

        #AK8963 registers
        self.AK8963_ADDR  = 0x0C
        self.AK8963_ST1   = 0x02
        self.HXH          = 0x04
        self.HYH          = 0x06
        self.HZH          = 0x08
        self.AK8963_ST2   = 0x09
        self.AK8963_CNTL  = 0x0A
        
        self.bus = smbus.SMBus(1)
        
        '''
         Set the sample rate (stability).
         By default the Gyro sampling frequecny is 8kHz.
         Inorder to change the sam_freq, gyro must be first configured to sample the data at 1kHz.
         To do that, set the DLF_Config flag which is found in the CONFIG register. 
         For more information, read the register map of both, ie. MPU9250 and MPU9250 for clarity
         Regiter map [pg 12,13]
        '''
        
        #sample rate = 1 kHz/(1+samp_rate_div)
        self.samp_rate_div = 4 
        self.bus.write_byte_data(self.MPU9250_ADDR, self.SMPLRT_DIV, self.samp_rate_div)
        time.sleep(0.1)

        # Reset all sensors
        self.bus.write_byte_data(self.MPU9250_ADDR, self.PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Power management and crystal settings
        self.bus.write_byte_data(self.MPU9250_ADDR, self.PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        # Write to Configuration register
        # Enables the DLF register flag.(More info in the register map) 
        self.bus.write_byte_data(self.MPU9250_ADDR, self.CONFIG, 0x01) 
        time.sleep(0.1)
        
        # Write to Gyro configuration register
        # Select scale --> [250.0,500.0,1000.0,2000.0] # degrees/sec --> might need to convert it into rad/sec  
        self.bus.write_byte_data(self.MPU9250_ADDR, self.GYRO_CONFIG, 0x00)
        time.sleep(0.1)

        # Write to Accel configuration register
        # Select scale --> [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)                            
        self.bus.write_byte_data(self.MPU9250_ADDR, self.ACCEL_CONFIG, 0x00)
        time.sleep(0.1)

        #Write to Maf configuration register 
        self.bus.write_byte_data(self.AK8963_ADDR, self.AK8963_CNTL, 0x00)
        time.sleep(0.1)
        #Set sampling 
        self.AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
        self.AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
        self.AK8963_mode = (self.AK8963_bit_res <<4)+ self.AK8963_samp_rate # bit conversion
        self.bus.write_byte_data(self.AK8963_ADDR,self.AK8963_CNTL,self.AK8963_mode)
        time.sleep(0.1)

        # interrupt register (related to overflow of data [FIFO])
        self.bus.write_byte_data(self.MPU9250_ADDR, self.INT_ENABLE, 1)
        time.sleep(0.1)
        
        print("Configuration Successful")
        
    def read_raw_bits(self, register):
        
        # Read the raw accel/gyro values 
        high = self.bus.read_byte_data(self.MPU9250_ADDR, register)
        low  = self.bus.read_byte_data(self.MPU9250_ADDR, register+1)
        
        # Combine high and low 
        # Shift MSB to the the left << 4 spaces and or it with LSB
        value = ((high << 8) | low)
        
        # convert to +- value
        if(value > 32768):
            value -= 65536
            
        return value
    
    def mag_read_raw_bits(self, register):
        
        # read magnetometer values
        low = bus.read_byte_data(self.AK8963_ADDR, register-1)
        high = bus.read_byte_data(self.AK8963_ADDR, register)
        
        # Combine high and low 
        # Shift MSB to the the left << 4 spaces and or it with LSB
        value = ((high << 8) | low)
        
        # convert to +- value
        if(value > 32768):
            value -= 65536
            
        return value
    
    def get_gyro(self, gyro_sens = 131.0):
        # Convert from deg/sec to rad/sec 
        gx = self.read_raw_bits(self.GYRO_XOUT_H) * np.pi/ (180 * gyro_sens)
        gz = self.read_raw_bits(self.GYRO_YOUT_H) * np.pi/ (180 * gyro_sens)
        gy = self.read_raw_bits(self.GYRO_ZOUT_H) * np.pi/ (180 * gyro_sens)
        
        return [gx, gy, gz]
    
    def get_accel(self, accel_sens = 16384.0, g = 9.80665):
        
        ax = self.read_raw_bits(self.ACCEL_XOUT_H) * g/ accel_sens
        ay = self.read_raw_bits(self.ACCEL_YOUT_H) * g/ accel_sens
        az = self.read_raw_bits(self.ACCEL_ZOUT_H) * g/ accel_sens
        
        return [ax, ay, -az]
    
    
    def get_mag(self, mag_sens = 6.826):
        
        loop_count = 0
        while 1:
            mx = self.mag_read_raw_bits(self.HXH) / (mag_sens)
            my = self.mag_read_raw_bits(self.HYH) / (mag_sens)
            mz = self.mag_read_raw_bits(self.HZH) / (mag_sens)

            # the next line is needed for AK8963
            if bin(bus.read_byte_data(self.AK8963_ADDR,self.AK8963_ST2))=='0b10000':
                break
                
            loop_count+=1
            
        return [mx, my, mz]
            
    def get_acc_angles(self):
        
        [ax, ay, az] = self.get_accel()
        roll  = np.arctan2(-ay, -az) * 180/ np.pi
        pitch = np.arctan2(ax, np.sqrt(ay ** 2.0 + az ** 2.0)) * 180/ np.pi
        
        return [roll, pitch]

