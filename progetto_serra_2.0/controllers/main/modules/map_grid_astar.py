import numpy as np

             
grid_maze_maschera= [['inf', 'X', 'X', 0,   0], ['inf', 0, 'X', 0,   0], ['inf', 0, 'X', 0,   0], ['inf', 0, 'X', 'X',   0],
                     ['inf', 'X',   0, 0,   0], ['inf', 0,   0, 0,   0], ['inf', 0,   0, 0,   0], ['inf', 0,   0, 'X',   0], 
                     ['inf', 'X',   0, 0,   0], ['inf', 0,   0, 0,   0], ['inf', 0,   0, 0,   0], ['inf', 0,   0, 'X',   0],
                     ['inf', 'X',   0, 0, 'X'], ['inf', 0,   0, 0, 'X'], ['inf', 0,   0, 0, 'X'], ['inf', 0,   0, 'X', 'X']]
             

             
def maschera_grid_maze(grid_maze_import, grid_maze_maschera):
    for i in range(16):
        for j in range(5): 
            if grid_maze_maschera[i][j] == 'inf':
                grid_maze_import[i][j] = 'inf'
            if grid_maze_maschera[i][j] == 'X':
                grid_maze_import[i][j] = 'X'
    return grid_maze_import


maze_2 = [[0, 0, 0, 0, 0, 0, 0],
          [0, 1, 0, 1, 0, 1, 0], # muro possibile
          [0, 0, 0, 0, 0, 0, 0],
          [0, 1, 0, 1, 0, 1, 0], # muro possibile
          [0, 0, 0, 0, 0, 0, 0],
          [0, 1, 0, 1, 0, 1, 0], # muro possibile
          [0, 0, 0, 0, 0, 0, 0]]


direzioni_lista = ['Visited', 'West', 'North', 'East', 'South']



def conversione_grid_maze(grid_maze_import_from_astar):
    grid_maze_import = grid_maze_import_from_astar
    grid_maze2 = maschera_grid_maze(grid_maze_import, grid_maze_maschera)
       
    for indice_cella in range(16):

        if 1 in grid_maze2[indice_cella]:
            for indice_direzione in range(5):
                
                if grid_maze2[indice_cella][indice_direzione] == 1:
                    print("muro nella cella:", indice_cella, "elemento_lista:",indice_direzione, " ossia, direzione: ", direzioni_lista[indice_direzione])
                    
                    
    ################################################
    #celle 0-1-2-3
    
    
                    if indice_cella == 0: #indice_cella da 0 a 7
                        if indice_direzione == 4: #indice_direzione == 4 -> sud
                            maze_2[1][0] = 1
                            maze_2[1][1] = 1
                            
                    if indice_cella == 1:
                        if indice_direzione == 4:
                            maze_2[1][2] = 1
                            maze_2[1][3] = 1
                            
                    if indice_cella == 2:
                        if indice_direzione == 4:
                            maze_2[1][4] = 1
                            maze_2[1][5] = 1
                            
                    if indice_cella == 3:
                        if indice_direzione == 4:
                            maze_2[3][6] = 1
                            #maze_2[3][7] = 1
    
    
    
                    if indice_cella == 0: #indice_cella da 0 a 7
                        if indice_direzione == 3: #indice_direzione == 3 -> est
                            maze_2[0][1] = 1
                            ###############
                            maze_2[1][1] = 1
    
                            
                    if indice_cella == 1:
                        if indice_direzione == 3:
                            maze_2[0][3] = 1
                            ###############
                            maze_2[1][3] = 1
    
                            
                    if indice_cella == 2:
                        if indice_direzione == 3:
                            maze_2[0][5] = 1
                            ###############
                            maze_2[1][5] = 1
                            
                            
                    if indice_cella == 1:
                        if indice_direzione == 1:
                            maze_2[0][1] = 1
                            #################
                            maze_2[1][1] = 1
                            
                    if indice_cella == 2:
                        if indice_direzione == 1:
                            maze_2[0][3] = 1
                            #################
                            maze_2[1][3] = 1
    
                            
                    if indice_cella == 3:
                        if indice_direzione == 1:
                            maze_2[0][5] = 1
                            #################
                            maze_2[1][5] = 1
                            
                            
    #################################################################################
     #celle 4-5-6-7
     
     
                    if indice_cella == 4: #indice_cella da 0 a 7
                        if indice_direzione == 2: #indice_direzione == 2 -> nord
                            maze_2[1][0] = 1
                            maze_2[1][1] = 1
                            
                    if indice_cella == 5:
                        if indice_direzione == 2:
                            maze_2[1][2] = 1
                            maze_2[1][3] = 1
                            
                    if indice_cella == 6:
                        if indice_direzione == 2:
                            maze_2[1][4] = 1
                            maze_2[1][5] = 1
                            
                    if indice_cella == 7:
                        if indice_direzione == 2:
                            maze_2[1][6] = 1
                            #maze_2[3][7] = 1
    
    
    
                    if indice_cella == 4: #indice_cella da 0 a 7
                        if indice_direzione == 4: #indice_direzione == 4 -> sud
                            maze_2[3][0] = 1
                            maze_2[3][1] = 1
                            
                    if indice_cella == 5:
                        if indice_direzione == 4:
                            maze_2[3][2] = 1
                            maze_2[3][3] = 1
                            
                    if indice_cella == 6:
                        if indice_direzione == 4:
                            maze_2[3][4] = 1
                            maze_2[3][5] = 1
                            
                    if indice_cella == 7:
                        if indice_direzione == 4:
                            maze_2[3][6] = 1
                            #maze_2[3][7] = 1
    
    

    
                    if indice_cella == 4: #indice_cella da 0 a 7
                        if indice_direzione == 3: #indice_direzione == 3 -> est
                            maze_2[2][1] = 1
                            #################
                            maze_2[3][1] = 1
                            maze_2[1][1] = 1
                            
                    if indice_cella == 5:
                        if indice_direzione == 3:
                            maze_2[2][3] = 1
                            ####################
                            maze_2[3][3] = 1
                            maze_2[1][3] = 1
    
                            
                    if indice_cella == 6:
                        if indice_direzione == 3:
                            maze_2[2][5] = 1
                            ##################
                            maze_2[3][5] = 1
                            maze_2[1][5] = 1
    
    
                            
                    if indice_cella == 4:
                        if indice_direzione == 1:
                            maze_2[2][1] = 1
                            #####################
                            maze_2[1][1] = 1
                            maze_2[3][1] = 1
    
                            
                    if indice_cella == 6:
                        if indice_direzione == 1:
                            maze_2[2][3] = 1
                            #####################
                            maze_2[1][3] = 1
                            maze_2[3][3] = 1
    
    
                            
                    if indice_cella == 7:
                        if indice_direzione == 1:
                            maze_2[2][5] = 1
                            #####################
                            maze_2[1][5] = 1
                            maze_2[3][5] = 1
    
                            
                    
    ######################################################################################
    #riga3, celle 8-9-10-11
                    if indice_cella == 8: #indice_cella da 0 a 7
                        if indice_direzione == 2: #indice_direzione == 2 -> nord
                            maze_2[3][0] = 1
                            maze_2[3][1] = 1
                            
                    if indice_cella == 9:
                        if indice_direzione == 2:
                            maze_2[3][2] = 1
                            maze_2[3][3] = 1
                            
                    if indice_cella == 10:
                        if indice_direzione == 2:
                            maze_2[3][4] = 1
                            maze_2[3][5] = 1
                            
                    if indice_cella == 11:
                        if indice_direzione == 2:
                            maze_2[3][6] = 1
    
    
    
                    if indice_cella == 8: #indice_cella da 0 a 7
                        if indice_direzione == 4: #indice_direzione == 4 -> sud
                            maze_2[5][0] = 1
                            maze_2[5][1] = 1
                            
                    if indice_cella == 9:
                        if indice_direzione == 4:
                            maze_2[5][2] = 1
                            maze_2[5][3] = 1
                            
                    if indice_cella == 10:
                        if indice_direzione == 4:
                            maze_2[5][4] = 1
                            maze_2[5][5] = 1
                            
                    if indice_cella == 11:
                        if indice_direzione == 4:
                            maze_2[5][6] = 1
                            #maze_2[3][7] = 1
    
    
    
                    if indice_cella == 8: #indice_cella da 0 a 7
                        if indice_direzione == 3: #indice_direzione == 3 -> est
                            maze_2[4][1] = 1
                            #########################
                            maze_2[3][1] = 1
                            maze_2[5][1] = 1
                            
    
                            
                    if indice_cella == 9:
                        if indice_direzione == 3:
                            maze_2[4][3] = 1
                            ############################
                            maze_2[3][3] = 1
                            maze_2[5][3] = 1
    
                            
                    if indice_cella == 10:
                        if indice_direzione == 3:
                            maze_2[4][5] = 1
                            ##########################
                            maze_2[3][5] = 1
                            maze_2[5][5] = 1
    
                            
                    if indice_cella == 9:
                        if indice_direzione == 1:
                            maze_2[4][1] = 1
                            ########################
                            maze_2[3][1] = 1
                            maze_2[5][1] = 1
    
                            
                    if indice_cella == 10:
                        if indice_direzione == 1:
                            maze_2[4][3] = 1
                            ##############################
                            maze_2[3][3] = 1
                            maze_2[5][3] = 1
    
                            
                    if indice_cella == 11:
                        if indice_direzione == 1:
                            maze_2[4][5] = 1
                            ##########################
                            maze_2[3][5] = 1
                            maze_2[5][5] = 1
                            
                            
    #########################################################################################
    #punti 12-13-14-15
    
    
    
                    if indice_cella == 12: #indice_cella da 0 a 7
                        if indice_direzione == 2: #indice_direzione == 2 -> nord
                            maze_2[5][0] = 1
                            maze_2[5][1] = 1
                            
                    if indice_cella == 13:
                        if indice_direzione == 2:
                            maze_2[5][2] = 1
                            maze_2[5][3] = 1
                            
                    if indice_cella == 14:
                        if indice_direzione == 2:
                            maze_2[5][4] = 1
                            maze_2[5][5] = 1
                            
                    if indice_cella == 15:
                        if indice_direzione == 2:
                            maze_2[5][6] = 1
                            #maze_2[3][7] = 1
    

    
                    if indice_cella == 12: #indice_cella da 0 a 7
                        if indice_direzione == 3: #indice_direzione == 3 -> est
                            maze_2[6][1] = 1
                            ###########################
                            maze_2[5][1] = 1
    
                            
                    if indice_cella == 13:
                        if indice_direzione == 3:
                            maze_2[6][3] = 1
                            ##############################
                            maze_2[5][3] = 1
    
                            
                    if indice_cella == 14:
                        if indice_direzione == 3:
                            maze_2[6][5] = 1
                            #############################
                            maze_2[5][5] = 1
    
    
                    if indice_cella == 13:
                        if indice_direzione == 1:
                            maze_2[6][1] = 1
                            ###########################
                            maze_2[5][1] = 1
    
                            
                    if indice_cella == 14:
                        if indice_direzione == 1:
                            maze_2[6][3] = 1
                            ##########################
                            maze_2[5][3] = 1
                            
                    if indice_cella == 15:
                        if indice_direzione == 1:
                            maze_2[6][5] = 1
                            ####################
                            maze_2[5][5] = 1
                            
    
    
    #########################################################################################
        else:
                print("nessun muro nella cella: ",indice_cella)
    
    return maze_2