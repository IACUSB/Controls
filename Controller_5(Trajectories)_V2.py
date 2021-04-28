###############################################################################
# Python3 script that uses RTI Connector for Python
# Take a look at the repo for examples and usage documentation:
#   https://github.com/rticommunity/rticonnextdds-connector-py
# MODIFIED VERSION -> Ansys - IAC Dec 2020

# import the required modules

from sys import path as sysPath
from os import path as osPath
import numpy as np
import matplotlib.pyplot as plt
import time, math
import rticonnextdds_connector as rti
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import sys
import math 

sys.setrecursionlimit(5000)

filepath = osPath.dirname(osPath.realpath(__file__))

def poly(lst, x):
    return lst[0]+lst[1]*x+lst[2]*x**2+lst[3]*x**3
        

# Set up your figure properties
fig_x=12
fig_y=5
fig=plt.figure(figsize=(fig_x,fig_y),dpi=120,facecolor=(0.8,0.8,0.8))
n=5
m=2
gs=gridspec.GridSpec(n,m)
p = [1, 5, 2, 6, 3, 4, 9]
T = np.arange(2000)
#print(T)

# call the RTI connector
connector = rti.Connector("SCADE_DS_Controller::Controller", filepath + "/Sensors_USB_5.xml")

# publisher::dataWriters
vehicleParamSet = connector.get_output("toVehicleCabToModelCorrectivePub::toVehicleCabToModelCorrectiveWriter")
vehicleParamSetSteering = connector.get_output("toVehicleSteeringCorrectivePub::toVehicleSteeringCorrectiveWriter")
DDSdone = connector.get_output("DDSDonePub::DDSDoneWriter")

# subscriber::dataReaders
vehicleParamGet = connector.get_input("vehicleStateVehicleOutputSub::vehicleStateVehicleOutputReader")
CameraPolinomGetF1 = connector.get_input("CameraRoadLinesPolinoms_Ego1_F1Sub::CameraRoadLinesPolinoms_Ego1_F1Reader")
CameraPolinomGetF2 = connector.get_input("CameraRoadLinesPolinoms_Ego1_F2Sub::CameraRoadLinesPolinoms_Ego1_F2Reader")
DDSwait = connector.get_input("DDSDoneSub::DDSDoneReader")

print("Controller started...") 
DDSdone.write()
speeds=[]

Refvel = 300      #[km/h]
gpa = 0

A = np.array([[0.9991, 1, 0],[0, 1, 0.01],[0, 0, 0]])
B = np.array([[0.1317],[0],[0]])
C = np.array([1, 0, 0])
L = np.array([[0.173437658157339],[0.281657842767891],[-22.313016014855567]])
xia = 0
xidota = 0
kpha = 100  
x = [5,10,15,20]

Refyaws = 0
K1 = 24 
K2 = 140

########## Vehicle Parameters ###############

I=1000
lf=2
lr=2
Caf=10000
Car=10000

while(True):  
    time.sleep(0.01)
    DDSwait.take()
    for DDSwaitSamples in DDSwait.samples.valid_data_iter:
        vehicleParamGet.take()
        CameraPolinomGetF1.take()
        CameraPolinomGetF2.take()        

        for vehicleParam in vehicleParamGet.samples.valid_data_iter:
            # speed is in meters per second, several options for conversion
            speed = math.sqrt((vehicleParam.get_number("cdgSpeed_x")**2) + (vehicleParam.get_number("cdgSpeed_y")**2) + (vehicleParam.get_number("cdgSpeed_z")**2))
            kph = speed*3.6
            speeds.append(speed)
            
            states = np.array([[kpha],[xia],[xidota]])
            inputs = np.array([[gpa],[kpha]])
            outputEst = np.dot((A-L*C),states) + np.dot(np.concatenate((B,L),axis=1),inputs)
            xiest = outputEst[1]
            xidotest = outputEst[2]
                        # Control law
            gp = (1/0.1317)*(-0.9991*kph-xiest+Refvel-0.980198673306755*(Refvel-kph)) 
            gp = float(gp)
            vehicleParamSet.instance.set_number("AcceleratorAdditive", gp)
            vehicleParamSet.instance.set_number("BrakeAdditive", 0.00)
            
            Sx = vehicleParam.get_number("cdgSpeed_x")
            Sy = vehicleParam.get_number("cdgSpeed_y")
            Pospsi = vehicleParam.get_number("cdgPos_heading")
            Yaw = vehicleParam.get_number("cdgPos_heading")
            YawS = vehicleParam.get_number("cdgSpeed_heading")  
            
        for vehicleParam in CameraPolinomGetF1.samples.valid_data_iter:            
            CurvatureRadio = vehicleParam.get_number("roadLinesPolynomsArray[0].curvatureRadius") 
            C0_R = vehicleParam.get_number("roadLinesPolynomsArray[1].c0")              
            C1_R = vehicleParam.get_number("roadLinesPolynomsArray[1].c1")              
            C2_R = vehicleParam.get_number("roadLinesPolynomsArray[1].c2")
            C3_R = vehicleParam.get_number("roadLinesPolynomsArray[1].c3")
            Coef_F1 = [C0_R,C1_R,C2_R,C3_R]

                   
        for vehicleParam in CameraPolinomGetF2.samples.valid_data_iter:            

            C0_L = vehicleParam.get_number("roadLinesPolynomsArray[0].c0")              
            C1_L = vehicleParam.get_number("roadLinesPolynomsArray[0].c1")              
            C2_L = vehicleParam.get_number("roadLinesPolynomsArray[0].c2")
            C3_L = vehicleParam.get_number("roadLinesPolynomsArray[0].c3")
            Coef_F2 = [C0_L,C1_L,C2_L,C3_L]
            
        if C0_L > 15:   
            P0 = -0.16885384538748482
            P1 = -0.11445216325864571
            P2 = -0.10167312110246307
            P3 = -0.12277951330902326
            P = 25
        else:
            P0 = ((poly(Coef_F2,x[0])-(poly(Coef_F1,x[0])))/2)+poly(Coef_F1,x[0])+5
            P1 = ((poly(Coef_F2,x[1])-(poly(Coef_F1,x[1])))/2)+poly(Coef_F1,x[1])+5
            P2 = ((poly(Coef_F2,x[2])-(poly(Coef_F1,x[2])))/2)+poly(Coef_F1,x[2])+5
            P3 = ((poly(Coef_F2,x[3])-(poly(Coef_F1,x[3])))/2)+poly(Coef_F1,x[3])+5
            P = 25
            
        L0 = ((P-x[1])*(P-x[2])*(P-x[3]))/((x[0]-x[1])*(x[0]-x[2])*(x[0]-x[3]))
        L1 = ((P-x[0])*(P-x[2])*(P-x[3]))/((x[1]-x[0])*(x[1]-x[2])*(x[1]-x[3]))
        L2 = ((P-x[0])*(P-x[1])*(P-x[3]))/((x[2]-x[0])*(x[2]-x[1])*(x[2]-x[3]))
        L3 = ((P-x[0])*(P-x[1])*(P-x[2]))/((x[3]-x[0])*(x[3]-x[1])*(x[3]-x[2]))
        Pref = L0*P0+L1*P1+L2*P2+L3*P3
        Yawe = math.atan(Pref/P)
            
        v = 0 - K1*(YawS-Refyaws) - K2*(-Yawe)

        delta = (I/(lf*2*Caf))*v-(-(-lf*2*Caf-lr*2*Car)/(lf*2*Caf*Sx))*Sy-((-lf**2*2*Caf-lr**2*2*Car)/(lf*2*Caf*Sx))*YawS
        vehicleParamSetSteering.instance.set_number("AdditiveSteeringWheelAngle", delta)
        vehicleParamSetSteering.write()

        print("Yaw " + str(round(Yaw,4)) + "  " + "Yaws " + str(round(YawS,4)) + "  " + "delta " + str(round(delta,4)) + "  " + "Yawe" + str(round(Yawe,4)) + " " + str(round(C0_L,4)))
        #print(str(round(C0_R,4)) + " "+ str(round(C1_R,4)) + " "+ str(round(C2_R,4)) + " "+ str(round(C3_R,4)) + " " + str(round(C0_L,4)) + " "+ str(round(C1_L,4)) + " "+ str(round(C2_L,4)) + " "+ str(round(C3_L,4)) + " " + str(round(Yawe,4)) + " " + str(round(Pref,4)))
            
        print(P0)
        print(P1)
        print(P2)
        print(P3)

        vehicleParamSet.write()

            #lagrange 
             
    
        #print("Gas pedal " + str(round(gp,4)) + " " + "Referencia " + str(round(Refvel,4)) + " " + "Velocidad  " + str(round(kph,4)))

        DDSdone.write()

