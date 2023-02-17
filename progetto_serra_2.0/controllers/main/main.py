"""main controller."""

import modules.robot_module as rb_m
import modules.slam_module as sl_m
import modules.map_grid as mp_m
import modules.navigation_module as nv_m
import modules.a_star_module as as_m

from controller import AnsiCodes

    
def main():
    
    ##### CREAZIONE DEL ROBOT E ISTANZA DEI SENSORI DI BORDO #####
    
    ROBOT = rb_m.RobotDevice()
    ROBOT.stopMotors()
    
    # recupero le istanze dei sensori del robot, qusti sono delle classi istanziate nella classe robot
    LIDAR = ROBOT.getInstanceLidar()
    EMITTER = ROBOT.getInstanceEmitter()
    RECEIVER = ROBOT.getInstanceReceiver()
    
    ###########################


    #typecontroller -> str di dimensione 3, (sup, opr) per indicare il tipo di robot e il corrispettivo "sub-controller"
    typeCntr = ROBOT.getTypeController()

    goalPosOp = -1
    
    typeOperation = ""
    
    destinatario = "broadcast"
    
    latoGriglia = 4     #lato della griglia
        
    lenMap = latoGriglia * latoGriglia      #numero di celle della mappa
    
    SLAM = None
    
    opInBase = True
    
    currentPosition = -1
    
    
        
    if typeCntr == "sup" :
        
        ##### ISTANZIO LA CLASSE SLAM PER LA MAPPATURA DELLA SCENA #####
        
        SLAM = sl_m.Slam(ROBOT, latoGriglia=latoGriglia)
        
        ###########################
        
        mappa = []
        
    
    
    while ROBOT.getInstanceRobot().step(ROBOT.getTimestep()) != -1:
    
    
        ################# SUPERVISORE ##################
    
        if typeCntr == "sup" :
            
            ROBOT.setPostazioneBase(1)
            
            #  LOGICA DEL SUPERVISORE:
            #   - MAPPATURA della scena tramite SLAM
            #   - GESTIONE MESSAGGI, invio e ricezione di richieste
            
            
            ############# SLAM ##############
            # se è la prima esecuzione oppure se non ci sono operazioni
            # disponibili da fare eseguire agli operai
            
            if( SLAM.getOperationFromOpList() == -1 and opInBase == True):
                
                print("Mappa conosciuta:")
                SLAM.MAP.printMapGrid()
                
                print("Eseguo lo SLAM")
                _, currentPosition = SLAM.slamRobot()
                
                print("OPERATION LIST", SLAM.getOperationList())

                print("Mappa aggiornata con gli ostacoli trovati:")
                SLAM.MAP.printMapGrid()
                
                currentPosition, _ = executeOperation(ROBOT, SLAM.MAP.getGridMap(), ROBOT.getPostazioneBase(), "RETURNED_BASE", currentPosition)
                
                # se c'è almeno una operazione da compiere e se la mappa è stata creata correttamente
                # invia la mappa e l'operazione da compiere
                if(SLAM.getOperationFromOpList()!=-1 and len(SLAM.MAP.getGridMap())>0):
                    
                    goalPosOp = SLAM.getOperationFromOpList()
                    # EMITTER PER INVIARE la mappa aggiornata e rispondere a richieste,
                    # tra cui la mappa con l'operazione da seguire
                    
                    # invia la mappa se siamo al primo step, quindi dopo la slam, o se gli operai
                    # hanno comunicato di aver completato i lavori e non ci 
                    # sono altri lavori da completare nella operationList
                    
                    destinatario = "broadcast"
                    
                    print(AnsiCodes.GREEN_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"], invio la mappa aggiornata a ROBOT[' + destinatario + "]" + AnsiCodes.RESET)
                    sendMessageWithEmitter(ROBOT, EMITTER, destinatario, "SEND_MAP", -1, SLAM.MAP.getGridMap())
                
            
            ############# END SLAM ##############
            
            
            ############# COMUNICAZIONE EMITTER - RECEIVER ##############
            
            ############# RECEIVER ##############       
            
            # recupera ad ogni timestep la coda dei messaggi ricevuti per
            # vedere se ci sono nuovi messaggi da leggere
            packets = RECEIVER.getPackets(lenMap * 5)
            
            if(len(packets) > 0):
                
                for packet in packets:
                    
                    message = {
                          "id_message" : packet[0],
                          "mittente" : packet[1],
                          "destinatario" : packet[2],
                          "operazione" : packet[3],
                          "goalPosOp"    : packet[4],
                          "time_send" : packet[5],
                          "mappa" : packet[6]
                    }
                    
                    print(AnsiCodes.RED_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"] ha ricevuto il seguente messaggio: '  + AnsiCodes.RESET)
                    
                    print(AnsiCodes.BLUE_FOREGROUND)
                    
                    for key, data in message.items():
                        print("\t", key + ": " , data,  "\n")
                    
                    print(AnsiCodes.RESET)
                    
                    
                    if message["destinatario"] == "broadcast" or message["destinatario"] == ROBOT.getMyName() and len(SLAM.MAP.getGridMap())>0 :
                        
                        #se è passato troppo tempo scarta il messaggio
                        if message["time_send"] > ROBOT.getTimeRobot() + 30:
                            print(f"Il messaggio con ID {message['id_message']} e' stato scartato")
                            continue
                        
                        # se il messaggio ricevuto contiene una richiesta da parte
                        # di un operaio di un lavoro da compiere, il supervisore
                        # recupera nel dictionary operationList, indicato di seguito
                        # operationList = {
                        #     "MAX": [],
                        #     "MEDIUM": [],
                        #     "MIN": []
                        # }
                        # la prima operazione trovata con massima priorità,
                        # e lo invia nel messaggio di risposta all'operaio
                        if(message["operazione"] == "SEND_ME_OP"): 
                            
                            # imposta come destinatario il robot che ha fatto la richiesta
                            destinatario = message["mittente"]
                            
                            goalPosOp = SLAM.getOperationFromOpList()
                            
                            #se non ci sono operazioni lo manda nella sua postazione
                            # che abbiamo imposto in alto a destra
                            if(goalPosOp == -1):
                                goalPosOp = SLAM.MAP.getLatoGrid()
                                typeOperation = "NO_OP"
                                print(AnsiCodes.GREEN_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"], per il momento non ci sono altre operazioni da svolgere,  ROBOT[' + destinatario + '] torna alla base, cella [' , goalPosOp , ']' + AnsiCodes.RESET)
                            else:
                                typeOperation = "OP_SENDED"
                            print(AnsiCodes.GREEN_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"], invio l\'operazione al ROBOT[' + destinatario + '] la cella da innaffiare e\' la [' , goalPosOp , ']' + AnsiCodes.RESET)
                            sendMessageWithEmitter(ROBOT, EMITTER, destinatario, typeOperation, goalPosOp, SLAM.MAP.getGridMap())
                            
                            
                        if(message["operazione"] == "RETURNED_BASE"):
                            # imposta come destinatario il robot che ha fatto la richiesta
                            destinatario = message["mittente"]
                            
                            #recupera l'operazione con massima priorità nella coda delle operazioni
                            goalPosOp = SLAM.getOperationFromOpList()
                            
                            #se c'è almeno una operazione la manda all'operaio
                            if(goalPosOp != -1):
                                typeOperation = "OP_SENDED"
                                print(AnsiCodes.GREEN_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"], invio l\'operazione al ROBOT[' + destinatario + '] la cella da innaffiare e\' la [' , goalPosOp , ']' + AnsiCodes.RESET)
                                sendMessageWithEmitter(ROBOT, EMITTER, destinatario, typeOperation, goalPosOp, SLAM.MAP.getGridMap())

                            else:   
                                # se non ci sono operazioni lo manda nella sua postazione, 
                                # e il supervisore ricomincia la mappatura per avere una
                                # mappa sempre aggiornata e per recuperare una nuova lista di operazioni da svolgere
                                print(AnsiCodes.GREEN_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"], procedo con una nuova mappatura per aggiornare la mappa e recuperare nuove operazioni da svolgere.' + AnsiCodes.RESET)
                                break
                            
                        if message["operazione"] == "OP_EXECUTED":
                            # imposta come destinatario il robot che ha fatto la richiesta
                            destinatario = message["mittente"]
                            
                            #recupera l'operazione con massima priorità dal messaggio inviato dall'operaio
                            goalPosOpCompleted = message["goalPosOp"]
                            
                            print(AnsiCodes.GREEN_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"], confermata esecuzione del lavoro da parte del ROBOT[' + destinatario + '], la elimino dalla lista.'  + AnsiCodes.RESET)
                            
                            # elimino l'operazione compiuta e confermata dall'operaio dalla lista delle operazioni
                            SLAM.delOperationFromOpList(goalPosOpCompleted)
                            
                            # recupero l'operazione successiva
                            goalPosOp = SLAM.getOperationFromOpList()
                            
                            if(goalPosOp != -1):
                                typeOperation = "OP_SENDED"
                                print(AnsiCodes.GREEN_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"], invio l\'operazione al ROBOT[' + destinatario + '] la cella da innaffiare e\' la [' , goalPosOp , ']' + AnsiCodes.RESET)
                                sendMessageWithEmitter(ROBOT, EMITTER, destinatario, typeOperation, goalPosOp, SLAM.MAP.getGridMap())
                            else:   
                                # se non ci sono operazioni lo manda nella sua postazione, 
                                # e il supervisore ricomincia la mappatura per avere una
                                # mappa sempre aggiornata e per recuperare una nuova lista di operazioni da svolgere
                                goalPosOp = SLAM.MAP.getLatoGrid()
                                typeOperation = "NO_OP"
                                print(AnsiCodes.GREEN_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"], per il momento non ci sono altre operazioni da svolgere,  ROBOT[' + destinatario + '] torna alla base, cella [' , goalPosOp , ']' + AnsiCodes.RESET)
                                sendMessageWithEmitter(ROBOT, EMITTER, destinatario, typeOperation, -1, SLAM.MAP.getGridMap())
                                break
                            
            ############# END RECEIVER ##############
            
            ############# END COMUNICAZIONE EMITTER - RECEIVER ##############
                    
    
       
        elif typeCntr == "op" :
            
        ################# OPERAIO ##################
            
            ROBOT.setPostazioneBase(4)
            
            mappa = []
            
            
            # QUI CI VA LA LOGICA DELL'OPERAIO, QUINDI SPOSTATI NEL PUNTO,
            # FAI L'OPERAZIONE NEL PUNTO.
            # INVIA MESSAGGI DI RICHIESTA, AD ESEMPIO LA MAPPA CON IL PERCORSO
            # AGGIORNATA DOPO AVER FINITO DI FARE L'OPERAZIONE
            
            
            ############# COMUNICAZIONE EMITTER - RECEIVER ##############
             
             
            ############# RECEIVER ##############       
            
            # # recupera ad ogni timestep la coda dei messaggi ricevuti per
            # # vedere se ci sono nuovi messaggi da leggere
            packets = RECEIVER.getPackets(lenMap * 5)
            
            if(len(packets) > 0):
                
                for packet in packets:
                    
                    message = {
                          "id_message" : packet[0],
                          "mittente" : packet[1],
                          "destinatario" : packet[2],
                          "operazione" : packet[3],
                          "goalPosOp"    : packet[4],
                          "time_send" : packet[5],
                          "mappa" : packet[6]
                    }
                    
                    #recupero la mappa dal messaggio ricevuto
                    mappa = message["mappa"]
                    
                    print(AnsiCodes.RED_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"] ha ricevuto il seguente messaggio: '  + AnsiCodes.RESET)
                    
                    print(AnsiCodes.BLUE_FOREGROUND)
                    
                    for key, data in message.items():
                        print("\t", key + ": " , data,  "\n")
                    
                    print(AnsiCodes.RESET)
                    
                    
                    if(message["destinatario"] == "broadcast" or message["destinatario"] == ROBOT.getMyName()):
                        
                        #se è passato troppo tempo scarta il messaggio
                        if message["time_send"] > ROBOT.getTimeRobot() + 30:
                            print(f"Il messaggio con ID {message['id_message']} e' stato scartato")
                            continue
                        
                        # se il messaggio ricevuto contiene una richiesta da parte
                        # di un operaio di un lavoro da compiere, il supervisore
                        # recupera nel dictionary operationList, indicato di seguito
                        # operationList = {
                        #     "MAX": [],
                        #     "MEDIUM": [],
                        #     "MIN": []
                        # }
                        # la prima operazione trovata con massima priorità,
                        # e lo invia nel messaggio di risposta all'operaio
                        if message["operazione"] == "OP_SENDED" or message["operazione"] == "NO_OP" and len(mappa)>0:
                            goalPosOp = message["goalPosOp"]
                            destinatario = message["mittente"]
                            
                            # se il tipo di messaggio è OP_SENDED, allora il messaggio contiene
                            # le informazioni per una nuova operazione da compiere, pertanto
                            # risponde di aver ricevuto il messaggio di operazione da compiere,
                            # e richiama la funzione per l'esecuzione del lavoro usando il
                            # goal_position presente nel messaggio ricevuto
                            if(message["operazione"] == "OP_SENDED"):
                                
                                print(AnsiCodes.GREEN_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"], confermo la ricezione dell\'operazione da eseguire da ROBOT[' + destinatario + '], vado in posizione nella cella [' , goalPosOp , ']'  + AnsiCodes.RESET)
                                
                                typeOperation = "OP_EXECUTED"
                                
                                # se il tipo di operazione è OP_EXECUTED allora vuol dire
                                # che l'operaio ha risposto che ha preso in carico l'operazione
                                # e quindi si può procedere con l'esecuzione
                                
                                currentPosition, mappaAstar = executeOperation(ROBOT, mappa, goalPosOp, typeOperation, currentPosition)
                                
                                sendMessageWithEmitter(ROBOT, EMITTER, destinatario, typeOperation, goalPosOp, mappa)
                                
                                # reimposto i valori di default
                                destinatario = "broadcast"
                               
                                
                                
                            # se il tipo di messaggio è NO_OP,
                            # allora la lista delle operazioni da compiere è vuota quindi
                            # il supervisore chiede di andare alla propria stazione base e
                            # successivamente inizierà la mappatura nuovamente per far
                            # verificare che non ci siano stati cambiamenti nella mappa
                            if(message["operazione"] == "NO_OP"):
                                
                                print(AnsiCodes.GREEN_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"], a quanto pare non ci sono piu\' operazioni da svolgere per il momento, torno alla base.'  + AnsiCodes.RESET)
                                
                                typeOperation = "RETURNED_BASE"
                                
                                # se il tipo di messaggio è RETURNED_BASE, allora vuol dire che
                                # l'operaio ha risposto e che sta tornando alla base
                                
                                currentPosition, mappaAstar = executeOperation(ROBOT, mappa, ROBOT.getPostazioneBase(), typeOperation, currentPosition)
                                
                                sendMessageWithEmitter(ROBOT, EMITTER, destinatario, typeOperation, ROBOT.getPostazioneBase(), mappa)
                                
                                # reimposto i valori di default
                                destinatario = "broadcast"
                               
                                
                            
                        # se il tipo di messaggio è SEND_MAP,
                        # vuol dire che è stata inviata la mappa aggiornata,
                        # l'operazione dovrà essere richiesta per poter inviare
                        # separatamente a più operai un operazione differente
                        if(message["operazione"] == "SEND_MAP" and len(mappa)>0):
                            typeOperation = "SEND_ME_OP"
                            destinatario = message["mittente"]
                            
                            print(AnsiCodes.GREEN_FOREGROUND + 'ROBOT [\"' + ROBOT.getMyName() + '\"], ricevuta la mappa aggiornata da ROBOT[' + destinatario + "]" + AnsiCodes.RESET)
                            
                            sendMessageWithEmitter(ROBOT, EMITTER, destinatario, typeOperation, -1, mappa)

                            # reimposto i valori di default
                            destinatario = "broadcast"
                        
                            
                ############# END RECEIVER ##############ù
            
            ############# END COMUNICAZIONE EMITTER - RECEIVER ##############
            
            # svuoto typeOperation
            typeOperation = ""
            
            
                     
def sendMessageWithEmitter(ROBOT, EMITTER, destinatario = "broadcast", typeOperation = "", goalPosOp = -1, mappa = []):
    ############# EMITTER ##############
            
    # EMITTER PER INVIARE la mappa aggiornata e rispondere a richieste,
    # tra cui il goal position con la posizione della cella finale
    # sulla quale bisogna eseguire un operazione
    if len(mappa)>0:
        packet = [ROBOT.getMyName(), destinatario, typeOperation, goalPosOp, ROBOT.getCurrentTime(), mappa]

        EMITTER.send(packet)
    else:
        print("Impossibile inviare il messaggio.")
        
    ############# END EMITTER ##############
    
    
    
def executeOperation(ROBOT, mappa, goal, operazione, currentPosition=4):
    
    # CREO un'istanza della classe MAP con la mappa ricevuta come argomento
    # e passo l'istanza di questa alla classe ASTAR che la userà per accedere
    # alla mappa passata dal supervisore, con i metodi di MAP
    
    ASTAR = as_m.Astar(ROBOT, mappa, currentPosition, goal, typeRobot= "operaio")
    
    pathFind = False
    mappaAstar = []
    
    pathFind, mappaAstar, currentPos = ASTAR.astarRobot()
    
    if(pathFind and operazione == "OP_EXECUTED"):
        ROBOT.innaffia(2)
    elif(operazione == "RETURNED_BASE"):
        print("[", ROBOT.getMyName(),"] Sono tornato alla stazione base.")
        
    # Nuova istanza della navigazione
    NAVIGATION = nv_m.Navigation(ROBOT, mappaAstar, typeRobot = "operaio")
    NAVIGATION.faceDirNorth()
        
    return currentPos, mappaAstar
    


if __name__ == "__main__":
    main()
    
    
