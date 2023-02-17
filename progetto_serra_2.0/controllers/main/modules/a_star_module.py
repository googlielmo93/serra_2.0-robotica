import numpy as np
from modules.a_star_module_utility import a_star
import modules.navigation_module as nv_m
import modules.map_grid as mp_m

from collections import deque

class Astar():
    
    ROBOT = None
    LIDAR = None
    MAP = None
    NAVIGATION = None
    
    matrixColors = []
    matrixLandmarks = []
    startPosition = -1
    goalPosition = -1
    goalPose = [-1, "North"]
    
    
    def __init__(self, robotInstance, mappa, startPosition=-1, goalPosition=-1, typeRobot="supervisore"):
        self.ROBOT = robotInstance
        self.LIDAR = self.ROBOT.getInstanceLidar()
        
        # crea una nuova istanza dalla mappa passata
        self.MAP = mp_m.MapGrid(mappa = mappa)
        
        # Nuova istanza della navigazione
        self.NAVIGATION = nv_m.Navigation(self.ROBOT, self.MAP, typeRobot = typeRobot)

        # definisce inizialmente le posizioni iniziale e di arrivo 
        # come quella della base di ricarica del robot operaio
        
        # se diverso da -1 allora assegna startPosition
        if(startPosition != -1):
            self.startPosition = startPosition
        else:
            self.startPosition = self.MAP.getLatoGrid()
        
        # se diverso da -1 allora assegna goalPosition
        if(goalPosition != -1):
            self.goalPosition = goalPosition
        else:
            self.goalPosition = self.startPosition
            
        # settiamo il valore goalPose che servirà per l'astar,
        # contiene la goalPosition e l'orientamento
        self.goalPose = [self.goalPosition, "North"]       

        # settiamo la posizione iniziale
        self.ROBOT.setNewPosition([self.convert_the_point_to_x_y(self.startPosition,'x'), self.convert_the_point_to_x_y(self.startPosition,'y')] )

        # settiamo l'ultima posizione letta dai sensori a [0, 0]
        self.ROBOT.setLastPosition([self.convert_the_point_to_x_y(self.startPosition,'x'), self.convert_the_point_to_x_y(self.startPosition,'y')])
        
        # settiamo la posa sulla mappa del robot
        self.MAP.setRobotPose([self.convert_the_point_to_x_y(self.startPosition,'x'), self.convert_the_point_to_x_y(self.startPosition,'y'), self.startPosition, 180])


    def convert_the_point_to_x_y(self, point_map, x_or_y):
        
        if x_or_y == 'x':
            coordinata = 0
        if x_or_y == 'y':
            coordinata = 1
        
        # matrice delle coordinate nella mappa
        x_y_matrix = np.array([[ [-15, 15],[-5, 15],[5, 15],[15, 15] ],
                        [ [-15,  5],[-5,  5],[5,  5],[15,  5] ],
                        [ [-15, -5],[-5, -5],[5, -5],[15, -5] ],
                        [ [-15,-15],[-5,-15],[5,-15],[15,-15] ]])
        
        latoGrid = self.MAP.getLatoGrid()

        # matrice degli indici
        indexCells = self.MAP.returnIndexCellsByNum(latoGrid)

        # recupera le coordinate nella gridMap a partire dalla cella
        coordinate_point = indexCells[point_map - 1]
        
        return x_y_matrix[coordinate_point[0]][coordinate_point[1]][coordinata]


    def getStartPosition(self):
        return self.startPosition
    
    def getGoalPosition(self):
        return self.goalPosition
    
    def getGoal_pose(self):
        return self.goalPose
    
    def setGoal_pose(self, goal, direction):
        self.goalPose = [goal, direction]
    
    def getGoalPoseByIndex(self, index):
        return self.goalPose[index]

    def wave_front(self, goal_cell):

        goal_cell_ind = goal_cell - 1
        start_cell_ind = self.MAP.getRobotPoseByIndex(2) - 1
        
        # segna il goal cell
        
        # imposta il conteggio del wave per iniziare dal valore successivo all'obiettivo
        count = self.MAP.getGridMapValueByIndex(goal_cell_ind, 0) + 1
        
        print("Inizio wave front:")
        
        
        while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:
            print(80*"-")
            
            print(f"Wave count: {count}")

            print("Mappa corrente: ", self.MAP.printMapGridAstar())
            
            # se la cella di partenza è uguale al valore del 
            # count del wave, allora è stato appena raggiunto
            if self.MAP.getGridMapValueByIndex(start_cell_ind, 0) == count:
                break
                
            # incrementa il wave count
            count += 1
                        
            # effettuo l'algoritmo astar che mi ritornerà la mappa con i punti del percorso da seguire
            self.MAP.setGridMapByMap( a_star( self.MAP.getRobotPoseByIndex(2), self.getGoalPoseByIndex(0), self.MAP.getGridMap() ) )


    def plan_path(self, goal_cell):
        # stack per il path planning
        plan = deque() 
        
        # ottenere l'indice della cella iniziale e di quella finale
        goal_cell_ind = goal_cell - 1
        start_cell_ind = self.MAP.getRobotPoseByIndex(2) - 1
        
        # valore della cella iniziale
        start_val = self.MAP.getGridMapValueByIndex(start_cell_ind, 0)

        # current value
        current_val = start_val - 1
        
        # l'indice della cella che si sta esaminando per i suoi vicini
        current_cell = start_cell_ind
        
        while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:
            
            # lo spostamento dalla cella corrente alla cella successiva
            mov = ""
        
            print(80*"-")
            print(f"Time: {self.ROBOT.getTimeRobot()}")
            
            # ottiene le pareti che circondano la cella corrente

            walls = self.MAP.getGridMapValueByCell(current_cell)

            # controlla le celle intorno per il valore successivo
            if current_cell > 3:
                if self.MAP.getGridMapValueByIndex(current_cell - 4, 0) == current_val and walls[2] == 0:
                    mov = "forward"
            if current_cell >0 :
                if self.MAP.getGridMapValueByIndex(current_cell - 1, 0) == current_val and walls[1] == 0:
                    mov = "left"
            if current_cell < 15:
                if self.MAP.getGridMapValueByIndex(current_cell + 1, 0) == current_val and walls[3] == 0:
                    mov = "right"
            if current_cell <= 11:
                if self.MAP.getGridMapValueByIndex(current_cell + 4, 0) == current_val and walls[4] == 0:
                    mov = "down"   
            
            # aggiunge lo spostamento alla pila
            plan.append((mov, current_val+1, current_cell))
            
            # aggiorna la cella corrente in base allo spostamento
            if mov == "forward":
                current_cell -= 4
            elif mov == "left":
                current_cell -= 1
            elif mov == "right":
                current_cell += 1
            elif mov == "down":
                current_cell += 4
                
            
            # stampa le info dello stack
            print(f"plan of moves: {plan}")
            
            # aggiorna current_val
            current_val -= 1
            
            # se siamo sulla cella obiettivo, interrompe la pianificazione
            if current_cell == goal_cell_ind:
                break
                
        # restituire il piano una volta eseguiti gli spostamenti dall'inizio all'obiettivo
        return plan
    
        
    def execute_plan(self, plan):
        
        while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:
            print("Tempo Robot: ", self.ROBOT.getTimestep())
            
            # far sì che il robot sia rivolto verso nord, se non lo è già
            if self.ROBOT.getDirectionRobot() != "North":
                self.NAVIGATION.faceDirNorth()
                
            # apre la mossa più a sinistra nella deque del piano per ottenere la mossa corrente da fare
            mov = plan.popleft()
            print("plan: ", plan)
            
            #direzione, numero del nodo del percorso, punto della matrice(contando da zero) 
            print("mov( ossia la direzione, numero del nodo del percorso, punto della matrice(contando da zero) ): ", mov) 
            
            # stampa la mossa mov e ottiene la direzione del movimento
            m = mov[0]
            
            # se il robot è rivolto a North
            if self.ROBOT.getDirectionRobot() == "North":
                # utilizzare il movimento m per ruotare il robot di conseguenza
                if m == "left":
                    self.NAVIGATION.turnLeft()
                elif m == "right":
                    self.NAVIGATION.turnRight()
                elif m == "down":
                    self.NAVIGATION.turnRight()
                    self.NAVIGATION.turnRight()
                    
                # una volta che il robot è stato allineato secondo m,
                # viene spostato nella cella successiva secondo il piano del fronte d'onda
                self.NAVIGATION.move(10.0)

            
            # se il robot è nella cella di destinazione, goalCell
            if self.MAP.getRobotPoseByIndex(2) == self.getGoalPoseByIndex(0):
                # fa rivolgere il robot verso nord
                self.NAVIGATION.faceDirNorth()
                
                # fa ruotare il robot secondo la direzione indicata in goal_pose
                if self.getGoalPoseByIndex(1) == "West":
                    self.NAVIGATION.turnLeft()
                elif self.getGoalPoseByIndex(1) == "East":
                    self.NAVIGATION.turnRight()
                elif self.getGoalPoseByIndex(1) == "West":
                    self.NAVIGATION.turnRight()
                    self.NAVIGATION.turnRight()
                    
                # vede se il robot si è allineato alla direzione della posa obiettivo
                if self.ROBOT.getDirectionRobot() == self.getGoalPoseByIndex(1):
                    break
        return 0        


    def astarRobot(self):
            
        while self.ROBOT.getInstanceRobot().step(self.ROBOT.getTimestep()) != -1:
            # altera la griglia in modo da poterla utilizzare con l'algoritmo astar per ottenere il percorso
            self.wave_front(self.getGoalPoseByIndex(0))
            
            # realizzare il piano dalla griglia alterata
            plan = self.plan_path(self.getGoalPoseByIndex(0))
            
            # esegui il plan
            val = self.execute_plan(plan)
            
            if val == 0:
                print(80*"-")
                print(f"| Time Robot: {self.ROBOT.getTimeRobot()}")
                print("| Obiettivo raggiunto, posso iniziare l'operazione di irrigazione.")                
                print("| Posizione finale: ", self.MAP.getRobotPoseByIndex(2))
                print(80*"-")
                self.MAP.resumeGridMapAstar()
                return True, self.MAP.getGridMap(), self.MAP.getRobotPoseByIndex(2)
     
