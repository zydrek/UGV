#codigo teste fase 1: controlar o m 
 
import time 
from base_ctrl import BaseController 
#codigo 1: mexer nos motores do robo 
#ligar o cerebelo do robo 
base = BaseController('/dev/ttyAMA0',115200) 
#ordem marcha 

base.base_json_ctrl({"T":13, "X":0.15, "Z":4.0}) 
#manter ação 
time.sleep(2) 
#paragem emergencia 
base.base_speed_ctrl(0,0)  
 
Código 2
