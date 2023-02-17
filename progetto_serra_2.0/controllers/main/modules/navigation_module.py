import random
import math
from collections import deque

class Navigation():
    
    ROBOT = None
    LIDAR = None
    MAP = None
    
    stack = None
    matrixColors = []
    matrixLandmarks = []
    
    priority = ("NONE", "MIN", "MEDIUM", "MAX")
    
    # La lista delle operazioni da compiere è un dictionary
    # con tre liste identificate dalla priorità MAX - MEDIUM - MIN
    #
    # operationList = {
    #     "MAX": [],
    #     "MEDIUM": [],
    #     "MIN": []
    # }
    operationList = {}
    
    typeRobot = "supervisore"
    
    
    def __init__(self, robotInstance, mappa, typeRobot="supervisore"):
        self.ROBOT = robotInstance
        self.LIDAR = self.ROBOT.getInstanceLidar()
        self.MAP = mappa
        
        # stack per la localizzazione
        self.stack = deque()
        
        # posizione dei landmark, x, y e orientamento q
        # yLandmark=0, rLandmark=1, gLandmark=2, bLandmark=3
        self.matrixLandmarks = [
            (-20.0, 20, 3.14),
            (20.0, 20, 3.14),
            (-20.0, -20, 3.14),
            (20.0, -20, 3.14)
        ]

        # matrice dei colori
        # Y=0, R=1, G=2, B=3
        self.matrixColors = [
            [1.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]
        
        #operationList è un dictionary
        self.operationList.update({
            "MAX": [],
            "MEDIUM": [],
            "MIN": []
        })
        
        self.setTypeRobot(typeRobot)
        
        
    def getTypeRobot(self):
        return self.typeRobot
    
    def setTypeRobot(self, t):
        self.typeRobot = t


    def convValPosSensorMtinIn(self, listVals):
        for i in range(len(listVals)):
            listVals[i] = math.fabs(listVals[i] * (self.ROBOT.getRadiusWheels()))
        return listVals
    
    def convValSensorMtinIn(self, listVals):
        for i in range(len(listVals)):
            listVals[i] = math.fabs(listVals[i] * 39.3701)
        return listVals
    
    def convMtoI(self, val):
        return val * 39.3701
        
    def getStack(self):
        return self.stack
    
    def getStackByIndex(self, index):
        return self.stack[index]
    
    def getLenStack(self):
        return len(self.stack)
    
    def setStack(self, value):
        self.stack.append(value)
        
    def clearStack(self):
        # svuota lo stack
        self.stack.clear()


    def updateRobot(self, rotating=False):
        # Indicato come (x,y,n,q), dove "x,y" rappresenta la posizione del robot in coordinate globali,
        # "n" rappresenta il numero di cella della griglia e "q " rappresenta l'orientamento globale del robot
        # rispetto al quadro di riferimento globale, ad esempio Pose = (11.5, 2.3, 8, 1.1).
        
        # stampa il tempo robot
        print(80 * "-")
        print(f"Tempo robot: {self.ROBOT.getTimeRobot()}")
        
        # ottieni le nuove misure dai sensori di posizione come lista
        valPosSensor = self.ROBOT.getListPositionSensorsValues()
        self.convValPosSensorMtinIn(valPosSensor)
        
        if(self.getTypeRobot()=="supervisore"):
            # ottieni i valori della distanza, gli ostacoli, le curve disponibili
            # e il conteggio delle curve disponibili
            distValues, obstacles, availableTurn, countObstacles = self.getDistValObsAvailable()

            # getLocalizedRobot() -> True se si è localizzato
            # all'interno della scena, o viceversa
            print(f"Localizzazione robot: {self.MAP.getLocalizedRobot()}")
        
        
        # logica a seconda che il robot sia già stato localizzato o meno
        if self.MAP.getLocalizedRobot() or self.getTypeRobot()=="operaio": # True -> Localizzato
            if not rotating:    # rotating False -> Non sta ruotando
                # getCoordinateRobot_x_y si aspetta una lista di due
                # valori come argomento, corrispondenti ai valori letti
                # dai sensori per l'odometria.
                # return una list con le coordinate in base ai valori
                # letti dai sensori.
                x, y = self.ROBOT.getCoordinateRobot_x_y(valPosSensor)
                
                # recuperiamo la cella corrente
                n, _, _ = self.MAP.getCurrentCell(x, y)
                
            else:   # rotating True -> Sta ruotando
                # recupero le coordinate e il numero di cella del robot,
                # tramite la sua posa nella griglia
                x, y, n, _ = self.MAP.getRobotPose()
                
            # aggiorna l'angolo del robot
            q = (self.ROBOT.getRollPitchYawImuByIndex(2) * 180) / 3.14159
            
            # aggiorna la posa del robot nella griglia
            self.MAP.setRobotPose([x, y, n, q])
            

        else:   # False -> Non Localizzato
                # se non si è localizzato nella mappa mettiamo come pose
                # dei valori standard iniziali
            
            # aggiorna l'angolo del robot
            q = (self.ROBOT.getRollPitchYawImuByIndex(2) * 180) / 3.14159
            
            # aggiorna la posa del robot nella griglia
            self.MAP.setRobotPose([0, 0, 0, q])
            
        
        # stampa la nuova posa
        print(f"Posa: {self.MAP.getRobotPose()}")
            
        # aggiorna la direzione in cui il robot è rivolto
        self.updateDirectionRobot()
        
        # stampa la cella corrente del robot e la direzione
        print(f"Cella corrente del robot: {self.MAP.getRobotPoseByIndex(2)}, Direction: {self.ROBOT.getDirectionRobot()}")
        
        if(self.getTypeRobot()=="supervisore"):    
            # stampa il distValues, cioè la distanza dall'ostacolo
            print(f"Distanza dall'ostacolo: left: {distValues[0]}, front: {distValues[1]}, right: {distValues[2]}")
            
        
            # stampa gli ostacoli rilevati dal lidar intorno al robot
            print(f"Ostacoli rilevati: left: {obstacles[0]}, front: {obstacles[1]}, right: {obstacles[2]}")
            self.printObstaclesOrientation(obstacles)
            
            # stampa le direzioni disponibili che non hanno ostacoli, 
            # quindi è l'inverso della lista obstacles[] contenente gli 
            # ostacoli rilevati.
            print(f"Direzioni disponibili per andare: left: {availableTurn[0]}, front: {availableTurn[1]}, right: {availableTurn[2]}")
                
        # stampa la mappa aggiornata
        #self.MAP.printMapGrid()
        

    def updateDirectionRobot(self):
        self.ROBOT.setDirectionByInertialUnit(self.MAP.getRobotPoseByIndex(3)) 


    def getAvailableTurns(self, obstacles):
        # uno 0 per un ostacolot significa che non c'è alcun ostacolo in quella 
        # direzione, quindi availableTurn, con valori nell'ordine 
        # [sinistra, fronte, destra] == 0, significa non disponibile, 
        # 1 significa disponibile, mentre count è il numero di curve disponibili
        countAvailableTurn = 0
        availableTurn = [0, 0, 0]
        
        for i in range(len(obstacles)):
            # se non c'è alcun ostacolo nella direzione
            # i-esima allora vi si può dirigere
            if obstacles[i] == 0:
                availableTurn[i] = 1
                countAvailableTurn += 1
            # se c'è un ostacolo, obstacles[i] == 1,
            # altrimenti è uguale a 0
        return availableTurn, countAvailableTurn
            
            
    def getTurnsTaken(self, availableTurn):
        # Le turn/curve che sono già state fatte o dove turn_available = 0,
        # non sono possibili. turn = 1 se il robot ha fatto
        # un movimento che lo porterebbe a quella cella da dove si trova
        # attualmente o se quel movimento non è possibile, se la svolta è
        # possibile da fare e non è stata fatta prima, verrà
        # lasciata uguale a 0
        count = 0
        svoltaFatta = [0, 0, 0]
        # availableTurn = [sx, front, dx] boolean
        for i in range(len(availableTurn)):
            if availableTurn[i] == 0:
                # lo spostamento non è possibile,
                # quindi viene segnato come una curva già fatta prima
                svoltaFatta[i] = 1
                count += 1
            elif availableTurn[i] == 1:
                # una svolta a sinistra dalla posizione corrente che è già stata fatta in precedenza
                if i == 0 and (self.MAP.getGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2)-2, 0) == 1 or self.MAP.getGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2)-2, 0) == 2):
                    svoltaFatta[0] = 1
                    count += 1
                # uno spostamento in avanti dalla posizione attuale che è già stata svoltaFatta in precedenza
                
    ############### DA CONTROLLARE ##################

                # if i == 1 and self.MAP.getGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2)-9, 0) == 1:
                if i == 1 and (self.MAP.getGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2)-5, 0) == 1 or self.MAP.getGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2)-5, 0) == 2):
                    svoltaFatta[1] = 1
                    count += 1
                # a una curva a destra from current position that has already been svoltaFatta before
                if i == 2 and (self.MAP.getGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2), 0) == 1 or self.MAP.getGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2), 0) == 2):
                    svoltaFatta[2] = 1
                    count += 1
        return svoltaFatta, count    


    def checkObstacleBehind(self):
        # vede se un ostacolo si trova alle spalle della posizione corrente;
        # in caso affermativo, effettua una svolta a sinistra dalla posizione sud,
        # ora rivolta verso est, una volta che il robot è rivolto a sud ottiene 
        # i nuovi ostacoli e ottiene i valori di distanza, gli ostacoli, le curve 
        # disponibili e il conteggio delle curve disponibili.
        
        distValues, newObstacles, availableTurn, count = self.getDistValObsAvailable()
        self.ROBOT.setDirectionRobot("South")
        
        # mappa gli ostacoli con l'orientamento rivolto a sud
        self.addObstaclesToMapGrid(newObstacles)
                    
        # if turned around and a obstacle in the way 
        if newObstacles[1] == 1:
            if newObstacles[0] == 0:  # una curva a sinistra availableTurn after facing south
                self.turnLeft()
                self.ROBOT.setDirectionRobot("East")
            else:
                # face north again
                self.turnLeft(-181)
                self.ROBOT.setDirectionRobot("North")
                self.ROBOT.stopMotors()
            return True, availableTurn
        return False, availableTurn

    # ricava gli ostacoli intorno al robot dalle misure
    # lette dal sensore lidar lungo le tre direzioni sx, fronte, dx
    def getObstaclesAroundRobot(self, distValues):
        # left, front, right
        obstacles = [0, 0, 0]
        for i in range(3):
            if distValues[i] < 10.0:
                obstacles[i] = 1
        return obstacles


    def printObstaclesOrientation(self, obstacles):
        dirRobot = self.ROBOT.getDirectionRobot()
        if dirRobot == "North":
            print(f"West: {obstacles[0]}, North: {obstacles[1]}, East: {obstacles[2]}, South: 0")
        
        elif dirRobot == "South":
            print(f"West: {obstacles[2]}, North: 0, East: {obstacles[0]}, South: {obstacles[1]}")
        
        elif dirRobot == "West":
            print(f"West: {obstacles[1]}, North: {obstacles[2]}, East: 0, South: {obstacles[0]}")
        
        elif dirRobot == "East": 
            print(f"West: 0, North: {obstacles[0]}, East: {obstacles[1]}, South: {obstacles[2]}")


    # orientamento degli ostacoli secondo la list -> [left, front, right]
    def addObstaclesToMapGrid(self, obstacles):
        dirRobot = self.ROBOT.getDirectionRobot()
        
        if dirRobot == "North":
            if obstacles[0] == 1:  # left obstacle
                self.MAP.setGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2)-1, 1, 1)
                
            if obstacles[1] == 1:  # front obstacle
                self.MAP.setGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2)-1, 2, 1)
                
            if obstacles[2] == 1:  # right obstacle
                self.MAP.setGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2)-1, 3, 1)
                
        if dirRobot == "South":
            if obstacles[0] == 1:  # left obstacle
                self.MAP.setGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2)-1, 3, 1) # left è a east
                
            if obstacles[1] == 1:  # front obstacle
                self.MAP.setGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2)-1, 4, 1) # front è a south
                
            if obstacles[2] == 1:  # right obstacle
                self.MAP.setGridMapValueByIndex(self.MAP.getRobotPoseByIndex(2)-1, 1, 1) # right è a west 
        
            
        # aggiunge gli ostacoli aggiunti alle celle di accompagnamento vicine
        for numCell in range(self.MAP.getNumCells()):
            # griglia è [Visitato, Ovest, Nord, Est, Sud]ù
            
            # se la cella corrente e la cella a destra condividono 
            # entrambe un ostacolo interno, se una cella è impostata a 1, 
            # vengono impostate entrambe a 1 per il loro ostacolo E/W
            if numCell < (self.MAP.getNumCells() - 1):
                if self.MAP.getGridMapValueByIndex(numCell, 3) != self.MAP.getGridMapValueByIndex(numCell+1, 1):
                    self.MAP.setGridMapValueByIndex(numCell, 3, 1)
                    self.MAP.setGridMapValueByIndex(numCell+1, 1, 1)
            else: # numCell = numCells
                if self.MAP.getGridMapValueByIndex(numCell, 1) != self.MAP.getGridMapValueByIndex(numCell-1, 3):
                    self.MAP.setGridMapValueByIndex(numCell, 1, 1)
                    self.MAP.setGridMapValueByIndex(numCell-1, 3, 1)
                    
            # se la cella corrente e la cella sottostante condividono entrambe un ostacolo interno,
            # se una cella è impostata a 1, imposta entrambe a 1 per il loro ostacolo N/S
            if numCell < (self.MAP.getNumCells() - self.MAP.getLatoGrid()):
                if self.MAP.getGridMapValueByIndex(numCell, 4) != self.MAP.getGridMapValueByIndex(numCell+4, 2):
                    self.MAP.setGridMapValueByIndex(numCell, 4, 1)
                    self.MAP.setGridMapValueByIndex(numCell+4, 2, 1)
            else: # numCell >= numCells - latoGrid and numCell < numCells - 1
                if self.MAP.getGridMapValueByIndex(numCell, 2) != self.MAP.getGridMapValueByIndex(numCell-4, 4):
                    self.MAP.setGridMapValueByIndex(numCell, 2, 1)
                    self.MAP.setGridMapValueByIndex(numCell-4, 4, 1)


    def getDistValObsAvailable(self):
        # appena il robot si trova al centro della cella
        # ed è di nuovo rivolto verso nord, viene rimappato
        distValues = self.LIDAR.getLidarVector()
        distValues = self.convValSensorMtinIn(distValues)
        
        print("LIDAR VECTOR [Sx, Front, Dx]: ", distValues)
        
        # recupera gli eventuali ostacoli presenti nei dintorni del robot
        obstacles = self.getObstaclesAroundRobot(distValues)
                
        # ottiene le curve disponibili per gli ostacoli rilevati
        availableTurn, countAvailableTurn = self.getAvailableTurns(obstacles)
        return distValues, obstacles, availableTurn, countAvailableTurn
        

    def lookLandmarkInCurCell(self, range):
        
        print (80*"#" + "\n\nCerco il landmark più vicino alla mia posizione attuale.\n\n" + 80*"#")
        
        colorLandmark = None
        
        # effettua una rotazione per vedere se un landmark
        # si trova nei pressi del robot, lo fa per un
        # lasso di tempo pari a seconds
        seconds = 6.5
        
        endTime = seconds + self.ROBOT.getTimeRobot()
        
        listVel = self.ROBOT.getRotationSpeedWheelsRad(359.5, seconds)
        
        while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:
            
            print(80*"-")
            print(f"Tempo Robot: {self.ROBOT.getTimeRobot()}")
            
            print("Sto cercando un landmark nei pressi della cella corrente.")
            
            # se il tempo robot di movimento è inferiore a quello
            # stimato per la rotazione allora esegue la logica
            if self.ROBOT.getTimeRobot() < endTime:
                
                # getRotationSpeedWheelsRad() -> recupera la velocità di
                # rotazione in radianti [rad],
                # gli passiamo come argomenti l'angolo di rotazione e i
                # secondi entro la quale deve ruotare
                
                # passiamo a setVelocityWheels() come argomento una lista
                # delle velocità delle ruote
                # return una lista con le velocità di rotazione
                self.ROBOT.setVelocityWheels(listVel)
                
                # Viene recuperato il colore del  landmark tramite la camera e le API fornite
                objsRec = self.ROBOT.getCamera().getRecognitionObjects()
                
                if len(objsRec) > 0:
                    # prendiamo il primo oggetto riconosciuto dalla camera
                    obj = objsRec[0]
                    
                    # Si ottiene l'angolo relativo degli oggetti, e la 
                    # distanza relativa tra il robot e il landmark
                    # che viene riconosciuto dalla camera
                    posCam = obj.get_position()
                    # [x, y, distRelative]
                    posRelative = math.fabs(self.convMtoI(posCam[2]))                
                    
                    print(f"Distanza relativa dal landmark: {posRelative}")
                    print("posRelative", posRelative, "range", range)
                    if posRelative < range:
                        # recupera il colore dell'oggetto riconosciuto
                        colorLandmark = obj.get_colors()
                        break
            else:
                self.ROBOT.stopMotors()
                break
            
        if colorLandmark is None:
            print("Landmark non trovato.")
            self.ROBOT.faceDir("North")
            
        return colorLandmark


    def localizedRobotLandmark(self):
        # range definisce l'intervallo di valori in cui cercare i landmark
        range = 5.5
        startPointLocalize = [0,0]
        
        # ritorna il landmark trovato durante la ricerca per rotazione
        landmark = self.MAP.getLandmark(self.lookLandmarkInCurCell(range))
        
        # se non c'è nessun landmark all'interno del range
        if landmark is None:
            
            while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:
                
                # Viene recuperato il colore del  landmark tramite la camera
                objsRec = self.ROBOT.getCamera().getRecognitionObjects()
                
                if len(objsRec) > 0:
                    # prendiamo il primo oggetto riconosciuto dalla camera
                    obj = objsRec[0]
                    
                    # Si ottiene l'angolo relativo degli oggetti, e la 
                    # distanza relativa tra il robot e il landmark
                    # che viene riconosciuto dalla camera
                    posCam = obj.get_position()
                    posRelative = math.fabs(self.convMtoI(posCam[2]))                
                    
                    # recupera il colore dell'oggetto riconosciuto
                    colorLandmark = obj.get_colors()
                    
                    # dal colore del cilindro, otteniamo le matrice
                    # corrispondente al landmark riconosciuto,
                    # e quindi la sua posizione all'interno della mappa
                    landmark = self.MAP.getLandmark(colorLandmark)
                    
                    if posRelative > range:    
                        self.navigateToLandmark()
                    else:
                        break
                else:  # se non è stato trovato alcun landmark
                    print("Non vedo nessun landmark, procedo con la navigazione..")
                    self.navigateToLandmark()
        
        # svuota lo stack
        self.clearStack()
            
        # ottiene le coordinate del punto di partenza per mappare gli ostacoli
        # ora che il robot si è localizzato accanto a un punto di riferimento
        # yLandmark=0, rLandmark=1, gLandmark=2, bLandmark=3
        if landmark == self.MAP.getMatrixLandmarksByIndex(0):
            startPointLocalize = [-15.0, 15.0]    # yLandmark
        if landmark == self.MAP.getMatrixLandmarksByIndex(1): 
            startPointLocalize = [15.0, 15.0]     # rLandmark
        if landmark == self.MAP.getMatrixLandmarksByIndex(2):
            startPointLocalize = [-15.0, -15.0]   # gLandmark
        if landmark == self.MAP.getMatrixLandmarksByIndex(3):
            startPointLocalize = [15.0, -15.0]    # bLandmark
            
        # settiamo a true la variabile localize per indicare che si è
        # localizzato all'interno della mappa usando i landmark
        self.MAP.setLocalizedRobot(True)
        
        return landmark, startPointLocalize
    
    
    def move(self, spostamento): 
        # compie una traslazione pari a spostamento
        timeSeconds = self.ROBOT.getTimeMove(spostamento, self.ROBOT.getMaxSpeed())
        
        end_time = timeSeconds + self.ROBOT.getTimeRobot()
        
        while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:

            self.updateRobot()
            
            if self.ROBOT.getTimeRobot() < end_time:
                
                velWheel = []
                
                for wheel in self.ROBOT.getDictWheels()['listWheels'].values():
                    velWheel.append(self.ROBOT.getMaxSpeed() / wheel['wheelRadius'])
                
                self.ROBOT.setVelocityWheels([velWheel[0], velWheel[1]])
            else:
                self.ROBOT.stopMotors()
                break
            
            
    # Rotazione di "degrees" gradi da effettuare in "seconds" secondi
    def rotate(self, degrees, timeSeconds, direction):
        
        #stima il tempo complessivo della rotazione
        endTime = timeSeconds + self.ROBOT.getTimeRobot()
        listSpeed = self.ROBOT.getRotationSpeedWheelsRad(degrees, timeSeconds)
        
        while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:
            # aggiorna e stampa i dettagli del robot, rotating = True
            self.updateRobot(rotating=True)
            
            # aggiorna i valori dell'ultima posizione
            # getListPositionSensorsValues --> restituisce una lista dei
            # valori ordinati secondo l'ordine nella dictionary
            listPosValues = self.convValPosSensorMtinIn(self.ROBOT.getListPositionSensorsValues())
            self.ROBOT.setLastPosition(listPosValues)

            print(f"Rotazione in direzione {direction}...")

            if self.ROBOT.getTimeRobot() < endTime:
                # con getRotationSpeedWheelsRad() recupero le velocità
                # di rotazione delle singole ruote e passa la list
                # con le velocità a setVelocityWheels() per settare
                # le velocità delle ruote
                self.ROBOT.setVelocityWheels(listSpeed)
            else:
                self.ROBOT.stopMotors()
                break  
        

    # ruota il robot di 90° verso SX
    def turnLeft(self, degrees=-90.5):
        rot = math.floor(math.fabs(degrees) / 90)
        sec = 1.5 * rot
        self.rotate(degrees, sec, "left")           
            
    # ruota il robot di 90° verso DX
    def turnRight(self, degrees=90.5):
        rot = math.floor(math.fabs(degrees) / 90)
        sec = 1.5 * rot
        self.rotate(degrees, sec, "right")
        
        
    # orienta verso il North e aggiorna il valore
    # della direzione del robot attuale
    def faceDirNorth(self):
        
        while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:
                 
            if self.ROBOT.getDirectionRobot() == "West":
                self.turnRight()
                
            elif self.ROBOT.getDirectionRobot() == "South":
                self.turnLeft(-181)
                
            elif self.ROBOT.getDirectionRobot() == "East":
                self.turnLeft()
                
            if self.ROBOT.getDirectionRobot() == "North":
                break
            
        self.ROBOT.setDirectionRobot("North")
        

    def navigateToLandmark(self):
        
        dirRobot = self.ROBOT.getDirectionRobot()
        
        # rivole innanzitutto il robot verso nord
        if dirRobot != "North":
            self.faceDirNorth()
        
        # ottiene i valori della distanza, gli ostacoli, le curve disponibili e il conteggio delle curve disponibili
        distValues, obstacles, availableTurn, countAvailableTurn = self.getDistValObsAvailable()

        # se non ci sono movimenti pendenti da fare allora procede con la logica seguente
        if len(self.getStack()) == 0:
        
            if countAvailableTurn == 0:
                # se l'unica opzione è quella di andare avanti perchè non ci sono curve disponibili
                # ed è già stato in quella cella, allora gira su se stesso di 180 gradi a destra per rivolgersi a sud
                self.turnRight(181)
                        
                # verifica se, dopo essersi rivolti a sud, c'è un ostacolo o meno
                dietroRobot, availableTurn = self.checkObstacleBehind()
                
                if dietroRobot:
                    if dirRobot == "East":
                        # guardando a sud, questa era una curva a sinistra
                        self.setStack("left")
                    elif dirRobot == "West":
                        # guardando a sud, questa era una curva a destra
                        self.setStack("right")
                else:
                    self.setStack("reverse")
                    
            elif availableTurn[1] == 1:
                self.setStack("forward")
                print("Mi sposto in avanti.")
                
            elif availableTurn[0] == 1:
                self.turnLeft()
                self.setStack("left")
                
            elif availableTurn[2] == 1:
                self.setStack("right")
                self.turnRight()
                
        else:  # se invece lo stack non è vuoto e quindi ci sono dei movimenti pendenti
        
            if availableTurn[1] == 1 and self.getStackByIndex(self.getLenStack()-1) != "reverse":
                self.setStack("forward")
                print("Mi sposto in avanti.")
                
            elif availableTurn[0] == 1 and self.getStackByIndex(self.getLenStack()-1) != "right":
                self.turnLeft()
                self.setStack("left")
                
            elif availableTurn[2] == 1 and self.getStackByIndex(self.getLenStack()-1) != "left":
                self.setStack("right")
                self.turnRight()
                
            else:
                countAvailableTurn = 0
                
            if countAvailableTurn == 0:
                # se l'unica opzione è in avanti ed è già stato in quella cella, gira su se stesso di 180 gradi a destra per andare verso sud
                self.turnRight(180)
                        
            # verifica se, dopo essersi rivolti a sud, c'è un ostacolo o meno
                dietroRobot, newAvailable = self.checkObstacleBehind()
                
                if dietroRobot:
                    if dirRobot == "East":
                        # guarda a sud, questa era una curva a sinistra
                        self.setStack("left")
                    elif dirRobot == "West":
                        # guardando a sud, questa era una curva a destra
                        self.setStack("right")
                else:  # nessun ostacolo dietro il robot, naviga verso sud
                    self.setStack("reverse")
                    
        self.move(10.0)
                
    # metodo per effettuare la navigazione                
    def navigationRobot(self):
        
        while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:
            
            # localizza il robot rispetto al landmark più vicino,
            # return il landmark riconosciuto e una list con la posizione
            # rispetto a questo
            landmark, positionCurRobot = self.localizedRobotLandmark()
            
            # settiamo a True la variabile localized
            self.MAP.setLocalizedRobot(True)
                
            # orienta il robot verso nord
            self.ROBOT.faceDir("North")
            
            # aggiorna la posa del robot nella mappa
            self.MAP.setRobotPose(
                [
                    positionCurRobot[0], positionCurRobot[1], 
                    self.MAP.getCurrentCell(positionCurRobot[0], positionCurRobot[1])[0], 
                    ((self.ROBOT.getRollPitchYawImuByIndex(2) * 180) / 3.14159)
                ]
            )
            
            # aggiorna la nuova posizione
            self.ROBOT.setNewPosition([positionCurRobot[0], positionCurRobot[1]])

                
        return self.MAP.getGridMap()        
        
                    
