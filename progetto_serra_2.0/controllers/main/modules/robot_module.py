"""robot module."""

from cmath import inf
import math

from controller import Robot, Motor, Camera, CameraRecognitionObject, InertialUnit

from modules.lidar_module import LidarDevice
from modules.receiver_module import ReceiverDevice
from modules.emitter_module import EmitterDevice
import modules.slam_module as slam



     
class RobotDevice(Robot):
    
    robot = None
    
    # wheels = {
    #   'listWheels' = {
    #       'nome ruota' = {
    #           'wheelInstance' = refInstanceDev
    #           'wheelDiameter' = float
    #           'wheelRadius' = float
    #           'wheelCinconference' = float
    #       } 
    #   },
    #   
    #   'diamAxis' = float
    #   'radiusAxis' = float
    # }
    wheels = {}
    wheelsName = ['left wheel motor', 'right wheel motor']
    
    devices = {}
    
    lidar = None
    emitter = None
    receiver = None
    camera = None
    imu = None
    wateringArm = None
    
    # positionSensors = {
    #   'listPositionSensors' = {
    #       'nome sensore' = {
    #           'positionSensorInstance' = refInstanceDev
    #       } 
    #   }
    # }
    positionSensors = {}
    positionSensorsName = ['left wheel sensor', 'right wheel sensor']
    
    encode_unit = 0.0
    max_speed = 9
    timestep = 32
    #impostiamo la direzione iniziale del robot
    directionRobot = "North"
    
    lastPosition = [0, 0]
    newPosition = [0, 0]
    
    postazioneBase = 0
    
    
    def __init__(self):
        
        self.robot = Robot()
        
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.wheels.update({'listWheels': {}})
        
        #wheels è un dictionary
        for i in range(len(self.wheelsName)):
            self.wheels['listWheels'].update({
                self.wheelsName[i] : {}
            })
        
        index=0
        #inserisce le info per la singola ruota
        for wheel in self.wheels['listWheels'].values():
            wheel.update({ 'wheelInstance' : self.robot.getDevice(self.wheelsName[index]) })
            wheel['wheelInstance'].setPosition(float('inf'))
            
            wheel.update({ 'wheelDiameter' : 0.050})
            wheel.update({ 'wheelRadius' : (0.023 * 39.3701)})
            wheel.update({ 'wheelCinconference' : 2 * 3.14 * wheel['wheelRadius']})
            index = index + 1
        
        self.wheels['diamAxis'] = (0.0455*2) * 39.3701
        self.wheels['radiusAxis'] = self.wheels['diamAxis'] / 2.0

            
        #Recupero e abilitazione dei sensori di posizione
        #ODOMETRIA
        #positionSensors è un dictionary
        
        self.positionSensors.update({'listPositionSensors': {}})
        
        for i in range(len(self.positionSensorsName)):
            self.positionSensors['listPositionSensors'].update({
                self.positionSensorsName[i] : {}
            })
        
        #inserisce le info per il singolo sensore di posizione
        index = 0
        for positionSensor in self.positionSensors['listPositionSensors'].values():
            positionSensor.update({ 'positionSensorInstance' : self.robot.getDevice(self.positionSensorsName[index]) })
            positionSensor['positionSensorInstance'].enable(self.getTimestep())
            index = index + 1


        #devices è un dictionary
        for i in range(self.robot.getNumberOfDevices()):
            self.devices[self.robot.getDeviceByIndex(i).getName()] = self.robot.getDeviceByIndex(i)
        
        ########### inizializziamo i sensori lidar, emitter, receiver, camera, inertial unit ###################
        if self.getDeviceByName('lidar') != None:
            self.lidar = LidarDevice(self.getDeviceByName('lidar'), self.getTimestep())
        
        if self.getDeviceByName('emitter') != None:
            self.emitter = EmitterDevice(self.getDeviceByName('emitter'))
            self.emitter.setChannel()
            
        if self.getDeviceByName('receiver') != None:
            self.receiver = ReceiverDevice(self.getDeviceByName('receiver'), self.getTimestep())
            self.receiver.setChannel()
            
        if self.getDeviceByName('camera1') != None:
            self.camera = self.getDeviceByName('camera1')
            self.camera.enable(self.getTimestep())
            self.camera.recognitionEnable(self.getTimestep())
        
        if self.getDeviceByName('inertial unit') != None:
            self.imu = self.getDeviceByName('inertial unit')
            self.imu.enable(self.getTimestep())
            
            
        if self.getDeviceByName('watering arm motor') != None:
            self.wateringArm = self.getDeviceByName('watering arm motor')
            self.wateringArm.setPosition(float('inf'))
            self.wateringArm.setVelocity(0)
            
        if self.getDeviceByName('endRangeSensor') != None:
            self.endRangeSensor = self.getDeviceByName('endRangeSensor')
            self.endRangeSensor.disable()
        


    def getInstanceRobot(self):
        return self.robot
    
    def getPostazioneBase(self):
        return self.postazioneBase
    
    def setPostazioneBase(self, pos):
        self.postazioneBase = pos
    
    def getTimestep(self):
        return self.timestep
    
    def getTimeRobot(self):
        return self.robot.getTime()
    
    def getInstanceLidar(self):
        return self.lidar
    
    def getInstanceEmitter(self):
        return self.emitter
    
    def getInstanceReceiver(self):
        return self.receiver
    
    def getInstanceInertialUnit(self):
        return self.imu
    
    def getInstanceWateringArm(self):
        return self.wateringArm
    
    def goWaterArm(self, velocity):
        self.getInstanceWateringArm().setVelocity(float(velocity))
    
    def stopWaterArm(self):
        self.getInstanceWateringArm().setVelocity(0)
        
    def getInstanceEndRangeSensor(self):
        return self.endRangeSensor
        
    def goEndRangeSensor(self, period):
        self.getInstanceEndRangeSensor().enable(period)
    
    def stopEndRangeSensor(self):
        self.getInstanceEndRangeSensor().disable()
    
    #return il dict dei sensori di distanza
    def getDictPositionSensors(self):
        return self.positionSensors
    
    # return una lista con le istanze per il singolo sensore di posizione.
    # positionSensors = {
    #   'listPositionSensors' = {
    #       'nome sensore' = {
    #           'positionSensorInstance' = refInstanceDev
    #       } 
    #   }
    # }
    def getListPositionSensorsInstances(self):
        listVal = []
        for positionSensor in self.positionSensors['listPositionSensors'].values():
            listVal.append(positionSensor['positionSensorInstance'])
        return listVal
    
    # return una lista con i valori letti dai singoli sensori di posizione
    def getListPositionSensorsValues(self):
        listPV = []
        for l in self.getListPositionSensorsInstances():
            # getValue() return il valore letto dal sensore di posizione
            # posizionato sul joint della ruota.
            listPV.append(l.getValue())
        return listPV
    
    #return il dictionary delle ruote del robot
    def getDictWheels(self):
        return self.wheels
    
    def getDiameterAxisWheels(self):
        return self.wheels['diamAxis']
    
    def getRadiusAxisWheels(self):
        return self.wheels['radiusAxis']
    
    def getRadiusWheels(self):
        return self.wheels['listWheels'][self.wheelsName[0]]['wheelRadius']
    
    def getDeviceByName(self, name):
        return self.devices.get(name)
    
    def getTypeController(self):
        if self.robot.getName().startswith("supervisore"):
            return "sup"
        elif self.robot.getName().startswith("operaio"):
            return "op"
        
    def getMyName(self):
        return self.robot.getName()
    
    def getCurrentTime(self):
        return self.robot.getTime()     #FLOAT
    
    def compareTime(self, time_1, time_2):
        return time_1 if time_1 > time_2 else time_2
    
    #return una lista contenti le coordinate dell'ultima posizione del robot
    def getLastPosition(self):
        return self.lastPosition
    
    #return una lista contenti le coordinate della nuova posizione del robot
    def getNewPosition(self):
        return self.newPosition
    
    def setLastPosition(self, lastPos):   #recupera l'ultima posizione del robot
        self.lastPosition[0], self.lastPosition[1] = lastPos
    
    def setNewPosition(self, newPos):   #recupera la nuova posizione del robot
        self.newPosition[0], self.newPosition[1] = newPos
        
    def setVelocityWheels(self, listVel):
        i=0
        for wheel in self.getDictWheels()['listWheels'].values():
            wheel['wheelInstance'].setVelocity(listVel[i])
            i=i+1

    def getRollPitchYawImu(self):
        return self.getInstanceInertialUnit().getRollPitchYaw()
    
    def getRollPitchYawImuByIndex(self, index):
        return self.getInstanceInertialUnit().getRollPitchYaw()[index]
    
    def getMaxSpeed(self):
        return self.max_speed
    
    def getCamera(self):
        return self.camera
    
    def getDirectionRobot(self):
        return self.directionRobot
    
    
    def setDirectionRobot(self, dir):
        self.directionRobot = dir
    
    
    # recupera la velocità di rotazione in radianti [rad]
    # return una lista con le velocità di rotazione
    def getRotationSpeedWheelsRad(self, degrees, timeSeconds):
        circle = 2 * math.pi * self.getRadiusAxisWheels()
        distance = (degrees / 360) * circle
        linearVelocity = distance / timeSeconds
        leftWheelSpeed = linearVelocity / self.getRadiusWheels()
        rightWheelSpeed = -1 * linearVelocity / self.getRadiusWheels()
        return [leftWheelSpeed, rightWheelSpeed]
    

    # return una lista con le coordinate [x,y] del robot usando i valori 
    # del Position Sensor “vals”
    def getCoordinateRobot_x_y(self, valPosSensorRobot):
        
        lastPos = self.getLastPosition()
        newPos = self.getNewPosition()
        setValDiff = False

        diff = []
        
        for i in range(len(lastPos)):
            diff.append( math.fabs(valPosSensorRobot[i] - lastPos[i]))
            if math.fabs(diff[i]) >= .3:
                setValDiff = True
        
        if setValDiff:
            for i in range(len(diff)):
                diff[i] = 0.3
                
                
        # differenza media delle ruote di destra e di sinistra
        sum=0.0
        diff_avg = 0.0
        
        for i in range(len(diff)):
            sum = sum + diff[i]
        
        if sum > 0:
            diff_avg = sum / len(diff)
        
        # x e y vengono settati a seconda della direzione
        # in cui si muove il robot
        x = newPos[0]
        y = newPos[1]
        
        if self.getDirectionRobot() == "North":
            y = y + diff_avg
        elif self.getDirectionRobot() == "West":
            x = x - diff_avg
        elif self.getDirectionRobot() == "East":
            x = x + diff_avg
        elif self.getDirectionRobot() == "South":
            y = y - diff_avg

        
        newPos = [x, y]
        
        # aggiorno i valori
        self.setLastPosition(valPosSensorRobot)
        
        # aggiorno la nuova posizione
        self.setNewPosition(newPos)
        
        return newPos
    
    
    # return la direzione (non globale), calcolata in base ai valori dell'imu
    def setDirectionByInertialUnit(self, imuVal):
        if (imuVal <= -135 and imuVal >= -180) or (135 <= imuVal <= 180):
            direction = "West"
        elif imuVal <= -45 and imuVal > -135:
            direction = "South"
        elif 45 <= imuVal <= 135:
            direction = "North"
        elif (-45 < imuVal <= 0) or (0 <= imuVal < 45):
            direction = "East"
        self.setDirectionRobot(direction)
    
    
    # correzzione dell'angolo del robot in base alla
    # direction passata come argomento e l'angolo attuale del robot
    def faceDir(self, direction="North"):
        
        #recupera l'angolo corrispondente alla direzione
        if direction == "North":
            dirAngle = 90.0
        elif direction == "West":
            dirAngle = 180.0
        elif direction == "East":
            dirAngle = 0.0
        elif direction == "South":
            dirAngle = -90.0
        
        #settiamo la velocità di base e la tollerenza
        speed = 0.2
        tolerance = 0.1
        
        while(self.robot.step(self.getTimestep()) != -1):
            # ci restituisce l'angolo rispetto a Est in gradi
            q = (self.getRollPitchYawImuByIndex(2) * 180) / 3.14159

            diff = dirAngle - q
            
            # modulo della differenza di angoli,
            # tra la direzione desiderata e quella rilevata del robot
            absDiff = math.fabs(diff)
            
            print(80*"-")
            
            print(f"| Tempo Robot: {self.getTimeRobot()}")
            print(f"| Angolo del robot rispetto a Est(0°), e' uguale a [gradi]: {q}")
            
            self.setDirectionByInertialUnit(q)
            print(f"| Direzione del robot attuale: {self.getDirectionRobot()}")
            print(f"| Rotazione verso: {direction}..")
            
            print(80*"-")
            
            # rallentamento della velocità o inversione della direzione
            # a seconda del valore dell'absDiff 
            if diff < 0 or (180 <= absDiff <= 270):
                if absDiff < 0.2:
                    speed = -0.1
                elif absDiff < 5:
                    speed = -0.2
                else:
                    speed = -2.0
            elif absDiff < 0.2:
                speed = 0.2
            elif absDiff < 5:
                speed = 0.2 
            elif absDiff < 180:
                speed = 0.2
                
            # se la tolleranza è stata raggiunta, allora si ferma
            if absDiff < tolerance:
                self.stopMotors()
                break
            else:
                self.setVelocityWheels([-1 * speed, speed])
        
        
    def getTimeMove(self, distance, speed):
        return distance / speed
        
                
    def stopMotors(self):  #arresta i motori
        self.setVelocityWheels([0.0, 0.0])

    def stopMotorsBySeconds(self, timeSeconds):  
        #arresta i motori per un intervallo di secondi pari
        #all'argomento seconds della funzione
        self.setVelocityWheels([0.0, 0.0])
        self.robot.step(timeSeconds)
        
    
    def innaffia(self, seconds):
        print("Mi fermo per eseguire l'operazione.")
        self.setVelocityWheels([0.0, 0.0])
        
        print("[" + self.getMyName() + "] sto innaffiando...")
        print(50*"<0 ")
        
        self.goWaterArm(3.10)
        self.goEndRangeSensor(seconds)
        
        self.getInstanceRobot().step(seconds*10000)

        self.stopEndRangeSensor()
        self.stopWaterArm()
            
        print("[" + self.getMyName() + "] ho completato l'operazione.")