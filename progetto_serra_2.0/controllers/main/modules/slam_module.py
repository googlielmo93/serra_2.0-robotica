import random
import modules.navigation_module as nv_m
import modules.map_grid as mp_m
import math
from collections import deque

class Slam():
    
    ROBOT = None
    MAP = None
    NAVIGATION = None
    
    
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
    
    
    def __init__(self, robotInstance, typeRobot = "supervisore", latoGriglia = 4):
        self.ROBOT = robotInstance
        ##### CREAZIONE MAPPA SE SUPERVISORE #####
        
        self.MAP = mp_m.MapGrid(latoGrid = latoGriglia)
        self.NAVIGATION = nv_m.Navigation(self.ROBOT, self.MAP, typeRobot = typeRobot)
        
        #operationList è un dictionary
        self.operationList.update({
            "MAX": [],
            "MEDIUM": [],
            "MIN": []
        })
        
    # SEZIONE PRIORITà RANDOMICA: 
    #   simula definendo una priorità tra MAX, MEDIUM, MIN, 
    #   la lettura dell'umidità del terreno, stabilendo il grado di 
    #   intervento da parte di un robot operaio e la zona da servire
    def getPriority(self):
        return self.priority
        
                    
    def randomPriority(self):
        # NONE - MIN - MEDIUM - MAX
        priorityList = random.choices(self.getPriority(),  weights=(200, 40, 30, 20 ), k=1)
        return priorityList[0]
    
    
    def randomAddListOp(self, cell):
        priority = self.randomPriority()
        
        if priority != "NONE" and (cell not in self.MAP.checkObj()):
            self.addOpToOperationList(priority, cell)
            
        return priority
    
    
    def getOperationList(self):
        return self.operationList
    
    
    def getOperationFromOpList(self):
        for priority in self.getOperationList().values():
            if(len(priority)>0):
                return priority[0]
        return -1
    
    def delOperationFromOpList(self, cell):
        for priority in self.getOperationList().values():
            if(len(priority)>0 and cell in priority):
                del priority[0]
                
    def delOperationFromOpListAll(self):
        for priority in self.getOperationList().values():
            if(len(priority)>0):
                del priority[:]
    
    
    def addOpToOperationList(self, priority, value):
        if priority != "NONE":
            self.operationList.get(priority).append(value)

    # END SEZIONE PRIORITà RANDOMICA
    
    
    # SEZIONE SLAM: 
    #   cerca le celle non visitate nella griglia,
    #   il modulo navigation_module si occupa dell'orientamento e
    #   dello spostamento del robot all'interno della scena


    # passa in rassegna tutte le celle per vedere se hanno uno 0,
    # una cella che non è stata ancora visitata, nell'elenco o meno
    def checkNewCellsToVisit(self):
        
        countCell = 0
        noCellVisit = True
        
        for cell in self.MAP.getGridMap():
            # se c'è anche un solo 0 nell'elenco, c'è una cella che non è stata ancora visitata
            if cell[0] == 0:
                
                if self.MAP.getGridMapValueByIndex(countCell, 1) == 1 and self.MAP.getGridMapValueByIndex(countCell, 2) == 1 and self.MAP.getGridMapValueByIndex(countCell, 3) == 1 and self.MAP.getGridMapValueByIndex(countCell, 4) == 1:
                    #[Visitato, Ovest, Nord, Est, Sud]
                    #se sei il robot operaio nella tua postazione non considero la cella chiusa
                    
                    if(countCell == 3):
                        self.MAP.setGridMapValueByIndex(countCell, 4, 0)
                        
                        #cella sotto
                        self.MAP.setGridMapValueByIndex(countCell+self.MAP.getLatoGrid(), 2, 0)
                        
                        # lo metto a 3 per dire che è un operaio
                        self.markCellVisitedObj(countCell, 3)
                        countCell = countCell + 1
                        continue  
                        
                    # lo metto a 2 per dire che è un oggetto
                    self.markCellVisitedObj(countCell, 2)
                    countCell = countCell + 1
                    continue
                
                noCellVisit = False  
                 
            countCell = countCell + 1
            
        # a questo punto tutte le celle hanno un 1, quindi tutte le celle sono state visitate, il robot dovrebbe fermarsi
        return noCellVisit      


    def markCellVisited(self, cell):
        self.MAP.setGridMapValueByIndex(cell-1, 0, 1)
        
    # setta a 2 per dire che è un obj per la stampa
    def markCellVisitedObj(self, cell, type):
        self.MAP.setGridMapValueByIndex(cell, 0, type)
        

    def mappatura(self):
        # mappa il percorso con gli ostacoli, 
        # dando priorità alle celle che non sono state visitate
        
        while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:
            # una volta che il robot è di nuovo al centro della cella ed è rivolto verso nord,
            # viene mappata la cella per ottenere i valori delle distanze, gli ostacoli,
            # le curve disponibili e il conteggio di queste.
            distValues, obstacles, availableTurn, count = self.NAVIGATION.getDistValObsAvailable()
            
            # vengono aggiunti alla mappa gli ostacoli rilevati
            self.NAVIGATION.addObstaclesToMapGrid(obstacles)
            
            # dopo che gli ostacoli sono stati aggiunti, si vede se il robot deve continuare
            # la mappatura o ha finito perchè non ci sono più celle da visitare
            if self.checkNewCellsToVisit():        
                break 
            
            # si ottengono le curve che sono già state fatte e il conteggio di queste
            svoltaFatta, svoltaFattaCount = self.NAVIGATION.getTurnsTaken(availableTurn)
            
            # viene definita una priorità alle celle non visitate e si definisce
            # una logica per la gestione delle curve disponibili
            if count == 0:
                # ruota su se stesso orientandosi verso sud
                self.NAVIGATION.turnRight(180)
                
                # verifica se, dopo essersi rivolti a sud, c'è un ostacolo o meno
                self.NAVIGATION.checkObstacleBehind()
                
            if count == 1:  # solo 1 mossa di svolta disponibile
                if svoltaFattaCount == 3 or (availableTurn[1] == 1 and svoltaFatta[1] == 1):
                    # l'unica opzione è quella di andare avanti essendo già stati in quella cella,
                    # quindi ruota su se stesso verso sud
                    self.NAVIGATION.turnRight(180)
                    
                    # verifica se, dopo essersi rivolti a sud, c'è un ostacolo o meno
                    self.NAVIGATION.checkObstacleBehind()
                
                elif availableTurn[0] == 1 and svoltaFatta[0] == 0:  # una curva a sinistra è disponibile
                    self.NAVIGATION.turnLeft()
                
                elif availableTurn[2] == 1 and svoltaFatta[2] == 0:  # una curva a destra è disponibile
                    self.NAVIGATION.turnRight()
                
                elif availableTurn[0] == 1 and svoltaFatta[0] == 1:  # una curva a sinistra è disponibile, ed è già stato visitato in precedenza
                    self.NAVIGATION.turnLeft()
                
                elif availableTurn[2] == 1 and svoltaFatta[2] == 1:  # una curva a destra è disponibile, ed è già stato visitato in precedenza
                    self.NAVIGATION.turnRight()
            
            elif count == 2:  # ci sono due possibilità
                if svoltaFattaCount == 3:
                    
                    # si rivolge a sud girando a destra di 180 gradi
                    self.NAVIGATION.turnRight(180)
                    
                    # verifica se, dopo essersi rivolti a sud, c'è un ostacolo o meno
                    self.NAVIGATION.checkObstacleBehind()
            
                elif svoltaFatta[1] == 0:   # va in avanti
                    print("Mi sposto in avanti.")
            
                elif svoltaFatta[0] == 0:   # ruota a sinistra
                    self.NAVIGATION.turnLeft()
            
                elif svoltaFatta[2] == 0:   # ruota a destra
                    self.NAVIGATION.turnRight()
            
            # passa alla cella successiva, muovendosi di 1 in direzione nord
            self.NAVIGATION.move(10.0)
            
            # segna la cella appena visitata nella matrice
            self.markCellVisited(self.MAP.getRobotPoseByIndex(2))
            
            # passo la cella visitata e randomicamente verrà inserita
            # o meno nella operationList, contenente le celle dove compiere un'azione di innaffiatura,
            # simulando condizioni di umidità del terreno in cui necessità l'intervento degli operai.
            priority = self.randomAddListOp(self.MAP.getRobotPoseByIndex(2))
            
            if priority != "NONE":
                print("Aggiunta la cella visitata nella lista delle operazioni da compiere con priorita' " + priority)
            
            # rimette il robot rivolto verso nord
            if self.ROBOT.getDirectionRobot() != "North":
                self.NAVIGATION.faceDirNorth()
                self.ROBOT.faceDir("North")

    # metodo per effettuare lo slam     
    def slamRobot(self):
        
        while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:
            
            # localizza il robot rispetto al landmark più vicino,
            # return il landmark riconosciuto e una list con la posizione
            # rispetto a questo
            landmark, positionCurRobot = self.NAVIGATION.localizedRobotLandmark()
            
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
            
            # ora che il robot sa in che posizione si trova, lo marca sulla mappa
            # e inizia la mappatura della griglia
            self.markCellVisited(self.MAP.getRobotPoseByIndex(2))
            self.mappatura()
            
            
            # controlla dopo ogni passo se il robot ha finito di mappare
            # se True allora c'è ancora qualche cella da visitare, viceversa
            # avendo finito fa un break uscendo dal ciclo
            if self.checkNewCellsToVisit():
                break  
                
        currentPosition = self.MAP.getRobotPoseByIndex(2)
            
        return self.MAP.getGridMap(), currentPosition        
