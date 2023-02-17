import numpy as np
from collections import deque
from modules.map_grid_astar import conversione_grid_maze

class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other):
        return self.position == other.position

# Questa funzione restituisce il percorso di ricerca
def return_path(current_node,maze):
    path = []
    no_rows, no_columns = np.shape(maze)
    # qui creiamo il labirinto dei risultati inizializzato con -1 in ogni posizione
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Restituisce il percorso invertito, in quanto è necessario mostrare il percorso dall'inizio alla fine
    path = path[::-1]
    start_value = 0
    # aggiorniamo il percorso da inizio a fine trovato da A-star serch con ogni passo incrementato di 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1
    return result


def search(maze, cost, start, end):
    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param maze:
        :param cost
        :param start:
        :param end:
        :return:
    """

    # Creare il nodo iniziale e finale con i valori inizializzati per g, h e f
    start_node = Node(None, tuple(start))
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, tuple(end))
    end_node.g = end_node.h = end_node.f = 0

    # Inizializzare sia l'elenco dei nodi ancora da visitare che quello dei nodi visitati, 
    # in questo elenco verranno inseriti tutti i nodi ancora da visitare per l'esplorazione. 
    # Da qui troveremo il nodo con il costo più basso da espandere successivamente
    yet_to_visit_list = []  
    # in questo elenco metteremo tutti i nodi già esplorati in modo da non esplorarli di nuovo
    visited_list = [] 
    
    # Aggiungere il nodo iniziale
    yet_to_visit_list.append(start_node)
    
    # Aggiunta di una condizione di stop. Questo serve a evitare un ciclo infinito
    # e a interrompere l'esecuzione dopo un numero ragionevole di passi.
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # Quali sono le caselle da cercare? Il movimento del serarca è
    # sinistra-destra-alto-basso (4 movimenti) da ogni posizione.

    move  =  [[-1, 0 ], # go up
              [ 0, -1], # go left
              [ 1, 0 ], # go down
              [ 0, 1 ]] # go right


    # 1) Otteniamo prima il nodo corrente confrontando tutti i costi di f e selezionando il nodo con il costo più basso per un'ulteriore espansione.
    #     2) Controllare se la massima iterazione è stata raggiunta o meno. Imposta un messaggio e interrompe l'esecuzione
    #     3) Rimuoviamo il nodo selezionato dall'elenco yet_to_visit e aggiungiamo questo nodo all'elenco visited
    #     4) Eseguire il test di Perofmr Goal e restituire il percorso, altrimenti eseguire i seguenti passi
    #     5) Per il nodo selezionato scoprire tutti i figli (usare move per trovare i figli)
    #         a) ottenere la posizione corrente per il nodo selezionato (questo diventa il nodo genitore per i figli)
    #         b) controllare se esiste una posizione valida (il confine renderà non validi alcuni nodi)
    #         c) se un nodo è un muro, ignorarlo
    #         d) aggiungere all'elenco dei nodi figli validi per il genitore selezionato
            
    #         Per tutti i nodi figli
    #             a) se il nodo figlio è presente nell'elenco dei nodi visitati, ignorarlo e provare con il nodo successivo
    #             b) calcolare i valori g, h e f del nodo figlio
    #             c) se il figlio è nell'elenco dei nodi ancora da visitare, ignorarlo
    #             d) altrimenti spostare il figlio nell'elenco di quelli ancora da visitare
    
    no_rows, no_columns = np.shape(maze)

    while len(yet_to_visit_list) > 0:
        
        # Ogni volta che si fa riferimento a un nodo dall'elenco yet_to_visit, 
        # il contatore dell'operazione limite viene incrementato.
        outer_iterations += 1    

        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
                
        # se si arriva a questo punto si ritorna al percorso come se non ci 
        # fosse una soluzione o il costo di calcolo fosse troppo alto.
        if outer_iterations > max_iterations:
            print ("Rinuncia al pathfinding per troppe iterazioni")
            return return_path(current_node,maze)

        # Togliere il nodo corrente dall'elenco ancora da visitare, aggiungerlo all'elenco visitato
        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        # verifica se l'obiettivo è stato raggiunto o meno, in caso affermativo restituisce il percorso
        if current_node == end_node:
            return return_path(current_node,maze)

        #Genera figli da tutte le caselle adiacenti
        children = []

        for new_position in move: 

            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Assicurarsi che sia nel raggio d'azione (controllare se è all'interno del perimetro del labirinto)
            if (node_position[0] > (no_rows - 1) or 
                node_position[0] < 0 or 
                node_position[1] > (no_columns -1) or 
                node_position[1] < 0):
                continue

            if maze[node_position[0]][node_position[1]] != 0:
                continue

            new_node = Node(current_node, node_position)

            children.append(new_node)

        for child in children:

            if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                continue

            child.g = current_node.g + cost
            
            child.h = (((child.position[0] - end_node.position[0]) ** 2) + 
                       ((child.position[1] - end_node.position[1]) ** 2)) 

            child.f = child.g + child.h

            if len([i for i in yet_to_visit_list if child == i and child.g > i.g]) > 0:
                continue
            
            yet_to_visit_list.append(child)


def a_star(punto_finale, punto_iniziale, grid_maze_da_convertire):
        
        maze = conversione_grid_maze(grid_maze_da_convertire)

        start = conversione_cella_coordinata(punto_iniziale)
        print ("Punto_Iniziale_preconversione: ", punto_iniziale, "dopo conversione: ", start)
        end = conversione_cella_coordinata(punto_finale)
        print ("Punto_Finale_preconversione: ",punto_finale, "post conversione", end)
        cost = 1
    
        path = search(maze, cost, start, end)
        print("path_senza_considerare_i_muri: ",path)
        
        grid_path2 = [  [path[0][0], path[0][2],path[0][4],path[0][6]],
                        [path[2][0], path[2][2],path[2][4],path[2][6]],
                        [path[4][0], path[4][2],path[4][4],path[4][6]],
                        [path[6][0], path[6][2],path[6][4],path[6][6]]]
        grid_path2_array = np.array(grid_path2)
        
        print("grid_path2_array", "\n", grid_path2_array)
        
        x,y = np.where(grid_path2_array == 2)
        
        if 2 in grid_path2_array:
            
            grid_path2_array[x,y]=3
            
        else:
            pass
    
        x,y = np.where(grid_path2_array == 0)
        if 0 in grid_path2_array:
            
            grid_path2_array[x,y]=2
            
        else:
            pass
        
    
        
        x,y = np.where(grid_path2_array == 4)   
        if 4 in grid_path2_array:
            
            grid_path2_array[x,y]=4
            
        else:
            pass
        
        x,y = np.where(grid_path2_array == 6)        
        if 6 in grid_path2_array:
            
            grid_path2_array[x,y]=5
            
        else:
            pass
        
        x,y = np.where(grid_path2_array == 8)
        if 8 in grid_path2_array:
            
            grid_path2_array[x,y]=6
            
        else:
            pass
        
        x,y = np.where(grid_path2_array == 10)        
        if 10 in grid_path2_array:
            
            grid_path2_array[x,y]=7
            
        else:
            pass
        
        x,y = np.where(grid_path2_array == 12)
        if 12 in grid_path2_array:
            
            grid_path2_array[x,y]=8
            
        else:
            pass
    
    
        x,y = np.where(grid_path2_array == 14)
        if 14 in grid_path2_array:
            
            grid_path2_array[x,y]=9
            
        else:
            pass
    
    
        x,y = np.where(grid_path2_array == 16)
        if 16 in grid_path2_array:
            
            grid_path2_array[x,y]=10
            
        else:
            pass
    
    
        x,y = np.where(grid_path2_array == 18)
        if 18 in grid_path2_array:
            
            grid_path2_array[x,y]=11
            
        else:
            pass
    
    
        x,y = np.where(grid_path2_array == 20)
        if 20 in grid_path2_array:
            
            grid_path2_array[x,y]=12
            
        else:
            pass
            
        x,y = np.where(grid_path2_array == 22)
        if 22 in grid_path2_array:
            
            grid_path2_array[x,y]=13
            
        else:
            pass
    
        x,y = np.where(grid_path2_array == 24)
        if 24 in grid_path2_array:
            
            grid_path2_array[x,y]=14
            
        else:
            pass
    
        x,y = np.where(grid_path2_array == 26)
        if 26 in grid_path2_array:
            
            grid_path2_array[x,y]=15
            
        else:
            pass
    
        x,y = np.where(grid_path2_array == 28)
        if 28 in grid_path2_array:
            
            grid_path2_array[x,y]=16
            
        else:
            pass
            
        ####################################################
        grid_maze = grid_maze_da_convertire
        ####################################################
        
        if grid_path2_array[0,0] != -1:
            grid_maze[0][0] = grid_path2_array[0,0]
            
        if grid_path2_array[0,1] != -1:
            grid_maze[1][0] = grid_path2_array[0,1]
            
        if grid_path2_array[0,2] != -1:
            grid_maze[2][0] = grid_path2_array[0,2]
            
        if grid_path2_array[0,3] != -1:
            grid_maze[3][0] = grid_path2_array[0,3]
            
        if grid_path2_array[1,0] != -1:
            grid_maze[4][0] = grid_path2_array[1,0]
            
        if grid_path2_array[1,1] != -1:
            grid_maze[5][0] = grid_path2_array[1,1]
            
        if grid_path2_array[1,2] != -1:
            grid_maze[6][0] = grid_path2_array[1,2]
            
        if grid_path2_array[1,3] != -1:
            grid_maze[7][0] = grid_path2_array[1,3]
            
        if grid_path2_array[2,0] != -1:
            grid_maze[8][0] = grid_path2_array[2,0]
            
        if grid_path2_array[2,1] != -1:
            grid_maze[9][0] = grid_path2_array[2,1]
            
        if grid_path2_array[2,2] != -1:
            grid_maze[10][0] = grid_path2_array[2,2]
            
        if grid_path2_array[2,3] != -1:
            grid_maze[11][0] = grid_path2_array[2,3]
            
        if grid_path2_array[3,0] != -1:
            grid_maze[12][0] = grid_path2_array[3,0]
            
        if grid_path2_array[3,1] != -1:
            grid_maze[13][0] = grid_path2_array[3,1]
            
        if grid_path2_array[3,2] != -1:
            grid_maze[14][0] = grid_path2_array[3,2]
            
        if grid_path2_array[3,3] != -1:
            grid_maze[15][0] = grid_path2_array[3,3]
        
        return grid_maze

 
def conversione_cella_coordinata(coordinate_da_convertire):
    if coordinate_da_convertire == 1:
        punto_convertito = [0*2,0*2]
    if coordinate_da_convertire == 2:
        punto_convertito = [0*2,1*2]
    if coordinate_da_convertire == 3:
        punto_convertito = [0*2,2*2]
    if coordinate_da_convertire == 4:
        punto_convertito = [0*2,3*2]
    if coordinate_da_convertire == 5:
        punto_convertito = [1*2,0*2]
    if coordinate_da_convertire == 6:
        punto_convertito = [1*2,1*2]
    if coordinate_da_convertire == 7:
        punto_convertito = [1*2,2*2]
    if coordinate_da_convertire == 8:
        punto_convertito = [1*2,3*2]
    if coordinate_da_convertire == 9:
        punto_convertito = [2*2,0*2]
    if coordinate_da_convertire == 10:
        punto_convertito = [2*2,1*2]
    if coordinate_da_convertire == 11:
        punto_convertito = [2*2,2*2]
    if coordinate_da_convertire == 12:
        punto_convertito = [2*2,3*2]
    if coordinate_da_convertire == 13:
        punto_convertito = [3*2,0*2]
    if coordinate_da_convertire == 14:
        punto_convertito = [3*2,1*2]
    if coordinate_da_convertire == 15:
        punto_convertito = [3*2,2*2]
    if coordinate_da_convertire == 16:
        punto_convertito = [3*2,3*2]

    return (punto_convertito)