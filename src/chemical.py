from gpiozero import MCP3008
import datetime

class PH:
    def __init__ (self):
        mid_cal = float(1500)
        low_cal = float(2030)
        high_cal = float(975)
        
        recent = float(7)
        date = datetime.datetime(2021, 5, 17)

        
pH = PH() 

class chemicalSubsystem:
    def __init__ (self):
        pass

    def setOpening (self, pos):
        pass

    def probePH (self):
        pH_voltage = MCP3008(channel=0)
        float voltage_mV = 0
        for (int i = 0; i < volt_avg_len; ++i) {
            voltage_mV += MCP3008(channel=0) / 1024.0 * 5000.0
        }
        voltage_mV /= volt_avg_len
    
    # convert voltage to pH value
        if (voltage_mV > pH.mid_cal) { # high voltage = low ph
            pH.recent = 7.0 - 3.0 / (pH.low_cal - pH.mid_cal) * (voltage_mV - pH.mid_cal)
        } else {
            pH.recent = 7.0 - 3.0 / (pH.mid_cal - pH.high_cal) * (voltage_mV - pH.mid_cal)
        }
        
        # print pH value and time
        while True:
            pH.date = datetime.datetime.now()
            print(pH.recent, pH.date)

    def probeORP (self):
        ORP_voltage = MCP3008(channel=1)
        pass

    def moveServo (self, angle):
        pass

    
