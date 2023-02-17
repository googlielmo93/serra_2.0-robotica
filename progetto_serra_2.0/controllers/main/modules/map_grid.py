import ctypes
import math


class MapGrid():
    
    # Configurazione della griglia, se una qualsiasi delle WNES è
    # impostata a 1, lì c'è un ostacolo/muro interno o esterno.
    # Le tuple interne sono definite dal primo elemento che indica
    # se quella cella è stata visitata durante la mappatura, mentre
    # durante la fase di pathfinding l'elemento nella matrice
    # corrispettiva assumerà come connotazione il peso della cella
    # nel percorso catturato.
    
    # Gli altri elementi sono le coordinate cartesiane.
    # Avremo quindi delle liste così composte ->
    # [Visitato, Ovest, Nord, Est, Sud]
    
    # N.B. -> Il perimetro viene considerato come recitanto per
    # delimitarne i confini, quindi vi sarà il flag nella matrice che
    # implementa la griglia della mappa, come se vi fosse una parete
    # interna di delimitazione.
    griglia = []
    
     # n è l'indice+1 delle combinazioni associate (riga,colonna) di seguito riportate
    # per esempio, (0,0) è riga=0, colonna=0, quindi n sarebbe l'indice di questo nell'elenco,
    # che è 0, quindi aggiungendo 1 avremo n=0+1=1
    indexCells = []
    
    # posa del robot da tenere x, y, numero di griglia n e orientamento theta
    # (rappresentato come q)
    robotPose = [0.0, 0.0, 0, 180]
    
    
    # booleano, se il robot non è stato localizzato
    # non emette informazioni sulla posa x,y
    localized = False
    
    # matrice dei colori
    matrixColors = []
    
    # matrice dei landmarks
    matrixLandmarks = []
    
    # numero di celle nella griglia
    numCells = 16
    
    # la matrice è quadrata
    latoGrid = 4
    
    
    def __init__(self, mappa = [], latoGrid = 0):   
        
        if(len(mappa)>0):
            self.setGridMapByMap(mappa)
            latoGrid = int(len(mappa) / 4)
            self.setLatoGrid(latoGrid)
            self.setNumCells(len(mappa))
            
            #consideriamo una griglia quadrata
            self.setIndexCellsByNum(latoGrid)
            
        else:
            self.setLatoGrid(latoGrid)
            self.setNumCells(latoGrid * latoGrid)
            
            #consideriamo una griglia quadrata
            self.setIndexCellsByNum(latoGrid)
            
            #creazione della mappa quadrata di dimensioni latoGrid X latoGrid
            self.setGridMap(latoGrid)
        
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
            [1.0, 0.8, 1.0],
            [1.0, 0.6, 0.0],
            [0.5, 1.0, 0.5],
            [1.0, 0.8, 1.0]
        ]
        
        
    def getGridMap(self):
        return self.griglia
    
    
    def getGridMapValueByIndex(self, elList, c):
        return self.griglia[elList][c]
    
    
    def getGridMapValueByCell(self, cell):
        return self.griglia[cell]
    
    def setGridMapByMap(self, map):
        self.griglia = map
    
    
    def setGridMapValueByIndex(self, elList, c, v):
        self.griglia[elList][c] = v
        
    def setNumCells(self, numCells):
        self.numCells = numCells
        
    def resumeGridMapAstar(self):
        for i in range(len(self.getGridMap())):
            for j in range(len(self.getGridMapValueByCell(i))):
                col = self.getGridMapValueByIndex(i,j)
                if  col == 'inf' or col == 'OBJ' or col == 'OP' or ((type(col) is int) and col > 1) :
                    self.setGridMapValueByIndex(i, j, 0)
                if col == 'X':
                    self.setGridMapValueByIndex(i, j, 1)
    
        
    def printMapGridAstar(self):
        print("__________________________________________")
        for i in range(4):
            x = i*4
            if (self.griglia[x][0] == 0):
                v1 = "?"
            elif (self.griglia[x][0] == 1):
                v1 = "V"
            else:
                v1 = str(self.griglia[x][0])
                
            if (self.griglia[x+1][0] == 0):
                v2 = "?"
            elif (self.griglia[x+1][0] == 1):
                v2 = "V"
            else:
                v2 = str(self.griglia[x+1][0])
                
            if (self.griglia[x+2][0] == 0):
                v3 = "?"
            elif (self.griglia[x+2][0] == 1):
                v3 = "V"
            else:
                v3 = str(self.griglia[x+2][0])
                
            if (self.griglia[x+3][0] == 0):
                v4 = "?"
            elif (self.griglia[x+3][0] == 1):
                v4 = "V"
            else:
                v4 = str(self.griglia[x+3][0])
            
            print("|  "+ str(self.griglia[x][2]) +"\t  " +str(self.griglia[x+1][2])+"\t  " +str(self.griglia[x+2][2])
                +"\t  " +str(self.griglia[x+3][2])+ "    |")
                
            print("|" +str(self.griglia[x][1]) + " " +v1+" " + str(self.griglia[x][3])+"\t" +str(self.griglia[x+1][1])+ " " +v2+" " + str(self.griglia[x+1][3])
                +"\t" +str(self.griglia[x+2][1])+ " " +v3+" " + str(self.griglia[x+2][3])
                +"\t" +str(self.griglia[x+3][1]) + " " +v4+" " + str(self.griglia[x+3][3]) +"  |")
                
            print("|  "+str(self.griglia[x][4]) +"\t  " +str(self.griglia[x+1][4])+"\t  " +str(self.griglia[x+2][4])
                +"\t  " +str(self.griglia[x+3][4])+"    |")
                
            if(i==3):
                print("|________________________________________|\n")
            else:
                print("|                                        |")
    
        
    
    def setGridMap(self, latoGrid):
        # griglia =  [Visited, West, North, East, South]

        numCells = self.getNumCells()
        
        for i in range(numCells):
            listApp = [0, 0, 0, 0, 0]

            #lato sinistro
            if ((i) - latoGrid) % latoGrid == 0:
                listApp = [0, 1, 0, 0, 0]
                
                #spigolo North-West
                if i == 0:
                    listApp = [0, 1, 1, 0, 0]
                
                #spigolo South-West
                if i + latoGrid >= numCells:
                    listApp = [0, 1, 0, 0, 1]
                    
            if i > 0 and i < latoGrid :
                listApp = [0, 0, 1, 0, 0]
                
            if i + latoGrid > numCells and i < numCells :
                listApp = [0, 0, 0, 0, 1]
                    
            #lato destro
            if ((i+1) - latoGrid) % latoGrid == 0:
                listApp = [0, 0, 0, 1, 0]
                
                #spigolo North-East
                if i == latoGrid-1:
                    listApp = [0, 0, 1, 1, 0]
                
                #spigolo South-East
                if i == numCells-1:
                    listApp = [0, 0, 0, 1, 1]
                    
            self.griglia.append(listApp)
                
    
    #crea la matrice degli indici a partire dal numero di celle passato  
    def setIndexCellsByNum(self, latoGrid):
        latoGrid = abs(latoGrid)
        # lavoriamo con matrici quadrate, in caso di numeri dispari sommiamo 1
        if  latoGrid % 2 != 0:
            latoGrid = latoGrid + 1
        # creiamo la matrice degli indici, i cui valori sono tuple di due valori
        # corrispondenti alla riga e alla colonna della cella
        
        self.indexCells = [(i, j) for i in range(latoGrid) for j in range(latoGrid)]
        
    #crea la matrice degli indici a partire dal numero di celle passato  
    def returnIndexCellsByNum(self, latoGrid):
        latoGrid = abs(latoGrid)
        if  latoGrid % 2 != 0:
            latoGrid = latoGrid + 1
        
        return [(i, j) for i in range(latoGrid) for j in range(latoGrid)]
        
    def getNumCells(self):
        return self.numCells
    
    def getLatoGrid(self):
        return self.latoGrid
    
    def setLatoGrid(self, latoGrid):
        return self.latoGrid
        
    def getIndexCells(self):
        return self.indexCells
    
    def getIndexCellsValues(self, row, col):
        return self.indexCells[row][col]
    
    # return la list contenente la posa corrente del robot all'interno della mappa
    def getRobotPose(self):
        return self.robotPose
    
    # return la list contenente la posa corrente del robot all'interno della mappa
    def getRobotPoseByIndex(self, index):
        return self.robotPose[index]
        
    def setRobotPose(self, newRobotPose):
        self.robotPose = newRobotPose
        
    def getLocalizedRobot(self):
        return self.localized
    
    def setLocalizedRobot(self, boolLoc):
        self.localized = boolLoc
    
        
    # dalle coordinate spaziali (x,y) recupero il numero di
    # riga e di colonna della matrice e il numero di cella
    def getCurrentCell(self, x, y):
                
        n = 0
        row = 0
        col = 0
        
        latoC = self.getLatoGrid()
        
        latoD = int(math.fabs(latoC * 10) / 2)

        count = 0

        # es -20 : 20 con lato = 4
        for i in range(-latoD, latoD, 10):
            count = count + 1
            if(i == latoD and y >= i and y <= i+10):
                row = latoC - count
                break
            elif(y >= i and y < i+10):
                row = latoC - count
                break
            
        count = 0
                
        for i in range(-latoD, latoD, 10):
            if(i == -latoD and x >= i and x <= i+10):
                col = count
                break
            elif(x > i and x <= i+10):
                col = count
                break
            count = count + 1
        
        
        for i in range(len(self.getIndexCells())):
            if self.getIndexCellsValues(i, 0) == row and self.getIndexCellsValues(i, 1) == col:
                n = i + 1
                break
        return n, row, col
    

    def convMapGridToMatrixC(self, map):
        # Esempio di map grid -> lista di liste di 5 interi
        # mappa = [       
        #         [0, 1, 1, 0, 0], [0, 0, 1, 0, 0], [0, 0, 1, 0, 0], [0, 0, 1, 1, 0],
        #         [0, 1, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 1, 0], 
        #         [0, 1, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 1, 0],
        #         [0, 1, 0, 0, 1], [0, 0, 0, 0, 1], [0, 0, 0, 0, 1], [0, 0, 0, 1, 1]
        # ]

        # viene creato un array di array, con nCells row e 5 colonne per ognuna
        arr = (ctypes.c_int * 5 * len(map))()

        for i, el in enumerate(map):
            arr[i][:] = el
            
        return arr
    
    
    def getMatrixColorByIndex(self, index):
        # Y=0, R=1, G=2, B=3
        return self.matrixColors[index]
    
    
    def getMatrixLandmarksByIndex(self, index):
        # yLandmark=0, rLandmark=1, gLandmark=2, bLandmark=3
        return self.matrixLandmarks[index]


    # recupera il landmark dal colore passato come argomento
    def getLandmark(self, color):
        if color is None:
            return None
        elif color == self.getMatrixColorByIndex(0):    # Y
            return self.getMatrixLandmarksByIndex(0)    # y_landmark
        
        elif color == self.getMatrixColorByIndex(1):    # R
            return self.getMatrixLandmarksByIndex(1)    # r_landmark
        
        elif color == self.getMatrixColorByIndex(2):    # G
            return self.getMatrixLandmarksByIndex(2)    # g_landmark
           
        elif color == self.getMatrixColorByIndex(3):    # B
            return self.getMatrixLandmarksByIndex(3)    # b_landmark
    
    
    def checkObj(self):
        countCell = 0
        listObj = []
        for cell in self.getGridMap():
            if cell[0] == 2 or cell[0] == 3:
                listObj.append(countCell+1)
            countCell = countCell + 1

        return listObj



    def printMapGrid(self):
        print("________________________________________")
        for i in range(4):
            x = i*4
            
            if (self.getGridMapValueByIndex(x,  0) == 0):
                v1 = "?"
            elif (self.getGridMapValueByIndex(x,  0) == 2):
                v1 = "OBJ"
            elif (self.getGridMapValueByIndex(x,  0) == 3):
                v1 = "OP"
            else:
                v1 = "V"
            if (self.getGridMapValueByIndex(x+1,  0) == 0):
                v2 = "?"
            elif (self.getGridMapValueByIndex(x+1,  0) == 2):
                v2 = "OBJ"
            elif (self.getGridMapValueByIndex(x+1,  0) == 3):
                v2 = "OP"
            else:
                v2 = "V"
            if (self.getGridMapValueByIndex(x+2,  0) == 0):
                v3 = "?"
            elif (self.getGridMapValueByIndex(x+2,  0) == 2):
                v3 = "OBJ"
            elif (self.getGridMapValueByIndex(x+2,  0) == 3):
                v3 = "OP"
            else:
                v3 = "V"
            if (self.getGridMapValueByIndex(x+3,  0) == 0):
                v4 = "?"
            elif (self.getGridMapValueByIndex(x+3,  0) == 2):
                v4 = "OBJ"
            elif (self.getGridMapValueByIndex(x+3,  0) == 3):
                v4 = "OP"
            else:
                v4 = "V"
                
            print("|  "+ str(self.getGridMapValueByIndex(x,  2)) +"\t  " +str(self.getGridMapValueByIndex(x+1,  2))+"\t  " +str(self.getGridMapValueByIndex(x+2,  2))
                +"\t  " +str(self.getGridMapValueByIndex(x+3,  2))+ "    |")
                
            print("|" +str(self.getGridMapValueByIndex(x,  1)) + " " +v1+" " + str(self.getGridMapValueByIndex(x,  3))+"\t" +str(self.getGridMapValueByIndex(x+1,  1))+ " " +v2+" " + str(self.getGridMapValueByIndex(x+1,  3))
                +"\t" +str(self.getGridMapValueByIndex(x+2,  1))+ " " +v3+" " + str(self.getGridMapValueByIndex(x+2,  3))
                +"\t" +str(self.getGridMapValueByIndex(x+3,  1)) + " " +v4+" " + str(self.getGridMapValueByIndex(x+3,  3)) +"  |")
                
            print("|  "+str(self.getGridMapValueByIndex(x,  4)) +"\t  " +str(self.getGridMapValueByIndex(x+1,  4))+"\t  " +str(self.getGridMapValueByIndex(x+2,  4))
                +"\t  " +str(self.getGridMapValueByIndex(x+3,  4))+"    |")
                
            if(i==3):
                print("|_______________________________________|\n")
            else:
                print("|                                       |")