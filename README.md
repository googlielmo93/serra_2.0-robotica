# Serra 2.0

Il progetto "Serra 2.0", vuole essere un'evoluzione del progetto
"Serra" presentato per la materia "Linguaggi e Traduttori"
nell'anno accademico 2020/2021.7

## Autori

- [La Mantia Vincenzo](https://github.com/googlielmo93)
- [Caruso Mario](https://github.com/warioevolution)


### Descrizione del progetto

Il progetto **Κήπουρομπότ**, nasce dalla volontà di progettare e realizzare, attraverso il simulatore [WeBots](https://www.cyberbotics.com/), l'evoluzione di una classica serra, gestita interamente da un sistema automatizzato per l'innaffiamento a zone del terreno, simulando la dove fosse necessario un intervento da parte di un robot adibito a questa procedura (robot operaio), in seguito alla segnalazione da parte di un altro robot (robot supervisore) che oltre a mappare il terreno ciclicamente per avere una mappa aggiornata il più costantemente possibile, comunica la necessità di intervento al primo robot operaio più vicino e disponibile che raccoglie la segnalazione e comunica il suo intento di intervenire. Nello specifico il robot supervisore invierà attraverso l'uso di un sensore (emitter) una richiesta e se questa troverà risposta con esito positivo invierà l'ultima mappa aggiornata a sua disposizione. Il robot operaio calcolerà quindi il percorso ottimale, in base alla sua attuale posizione, che dovrà percorrere per arrivare al punto target dove dovrà poi intervenire. Una volta compiuta l'operazione lo comunicherà al supervisore, e resterà in attesa di un nuovo obiettivo oppure di poter essere disimpegnato, in tal caso tornerà alla sua postazione base che ha una collocazione fissa nella mappa.

Il progetto è stato pensando ed implementato per poter essere esteso a una squadra di robot operai gestiti da un robot supervisore che si occupa di mappare, rilevare i punti nella mappa su cui bisogna intervenire, e assegnare i compiti ai robot operaio. Nel progetto implementato per questioni pratiche abbiamo usato una coppia di robot supervisore e operaio, ma il codice è stato sviluppato affinchè possa essere esteso con qualche minimo accorgimento.

Il progetto si rivolge a tutte quelle realtà agricole dove si desidera:
- un’agricoltura di precisione
  - ottimizzazione della produzione a parità di acqua e terreni
  - più sostenibile per l’ambiente
- applicazioni agricole su larga scala
- difficoltà nel reperimento della manodopera
- interventi tempestivi sulle piantagioni; ad esempio, si potrebbe inserire un sistema avanzato di riconoscimento di visione artificiale per bloccare sul nasce:
  - infestazioni da cocciniglia
  - erbe infestanti
  - etc...
  


### Scenario di lavoro
L’applicazione che abbiamo immaginato per i nostri robot è quella dove più robot lavorano per un fine comune: curare un’area agricola; in questo scenario, la collaborazione non ha una struttura flat ma abbiamo preferito un robot coordinatore in grado fare valutazioni ad alto livello così da assegnare i compiti specifici al robot giusto (che nel nostro progetto prevederà un robot operaio in grado di irrigare). È stato scelto questo modo di operare perché le attività da svolgere possono essere molto diverse tra loro e quindi occorrono robot specializzati in determinate attività.

Abbiamo, quindi, inserito due tipi di robot:
- Robot Supervisore
  - mapping l’ambiente di lavoro
  - stabilisce i compiti da fare
  - invia le comunicazioni al robot operaio contenete
    - la mappa dell’ambiente di lavoro
    - il compito da svolgere

- Robot “operaio”, è un robot specializzato in una determinata attività. Ad esempio, uno potrebbe irrigare, uno spruzzare diserbanti, uno raccogliere i frutti, uno individuare piante infestanti, etc...

  - Riceve le comunicazioni da parte del robot supervisore
    - Mappa area di lavoro
    - Compiti da svolgere
    - Definisce il percorso ottimo per raggiungere l’area di lavoro
    
  - A-Star
    - Raggiunge il punto di lavoro
    - Esegue il suo compito specifico (irrigazione, raccolta, etc...)
    
    
### Altre caratteristiche
Stabilito lo scenario di lavoro del nostro progetto, abbiamo stabilito che dovesse integrare:
- esplorazione dell’ambiente di lavoro ignoto
  - il robot non ha una conoscenza priori dell’ambiente dove deve operare, tranne il numero delle celle totali del world, deve quindi individuare:
    - spazi attigui non percorribili, ossia celle libere ma separate, ad esempio, da uno steccato
    - spazi occupati, ossia celle non percorribili
  - individuazione dei punti da lavorare e compiti da eseguire
- la collaborazione tra robot
  - I robot comunicano
    - Ogni robot dispone di un emettitore e di un ricevitore
  - si coordinato sul lavoro da fare
  - si scambiano informazioni sull’area di lavoro
- determinazione del cammino ottimale per raggiungere il punto da lavorare
  - basato sull’algoritmo A-star
  
### Scelte progettuali e Algoritmi utilizzati
Dopo avere definito cosa dovessero fare i nostri robot, è stato scelto di usare un approccio
modulare per scrivere il codice. Tale scelta è stata presa per garantire:
- Scalabilità
- Facilità di identificazione e di correzione dei bug
- Facilità di inserimento di nuove funzioni

Al nostro codice è stata data una struttura di questo tipo:<br>
<img src="https://user-images.githubusercontent.com/10675526/219764190-fc8602d9-0247-4ae7-a3d1-9e6fecd0e71b.png" width="500"/>

Prevede un file main.py che va a richiamare i diversi moduli a seconda dei compiti che devono essere svolti.
La gran parte del codice (main e moduli) è stata scritta per accettare in ingresso qualsiasi dimensione dell’ambiente di lavoro così da adattarsi alle diverse esigenze di dimensionamento.
Tuttavia, resta una parte del codice che è specifica per funzionare su arene 4x4 e 8x8.

Abbiamo scelto di scrivere il codice interamente in Python 3.10 perché:
- volevamo un linguaggio di programmazione orientato agli oggetti
- per la sua elevata leggibilità sia per la sua
- fornisce una libreria standard che gestisce in modo automatico la memoria
- può eseguire lo stesso codice su molteplici piattaforme.
Sono state integrate le librerie:
- Numpy
- Time
- Math
- Random
- Struct

I moduli Webots importati per il corretto funzionamento sono stati:
- Lidar
- Emitter
- Receiver
- Robot
- Camera
- CameraRecognitionObject
- InertialUnit
- PositionSensor

### Slam in Κήπουρομπότ
Il robot supervisore non integra un vero SLAM perché, gli algoritmi sono ritagliati sulle risorse disponibili.
Siamo, quindi, scesi a un compromesso progettuale che prevede l’utilizzo di una matrice, dove il robot:
- Conosce il numero di celle presenti nel mondo
- si muove di cella in cella ed esamina, attraverso il LIDAR a tiro corto se vi sono ostacoli
- Approssima la propria posizione rispetto alla mappa utilizzando degli alberi posti agli angoli della mappa, usati come landmark.

### Schema di funzionamento
<img src="https://user-images.githubusercontent.com/10675526/219767000-eca13da4-c191-48fa-9a50-589fa3364ffa.png" width="500"/>

### A-STAR in Κήπουρομπότ
Per il nostro progetto si è scelto di non usare librerie già esistenti per Python perché ci si è resi conto che quanto già presente non era adatto al nostro utilizzo.
Le librerie presenti in Python calcolano il percorso su una matrice dove le celle possono essere considerate o percorribili o non percorribili: <br>
<img src="https://user-images.githubusercontent.com/10675526/219767382-afc2ae00-3c65-429e-ad58-6af6e1defcc6.png" width="500"/>
<br><br><br>
<img src="https://user-images.githubusercontent.com/10675526/219767993-fc609220-cde2-46dc-bb41-633dcf0136a0.png" width="600" />
<br><br><br>
<img src="https://user-images.githubusercontent.com/10675526/219768304-c31ba352-34a2-4cef-b550-d370541dd3e7.png" width="600" />

### Meccanismo di funzionamento
L’Algoritmo A-Star viene avviato dalla funzione “def a_star2(punto_finale, punto_iniziale, mappa_da_convertire)” che:
- riceve in ingresso lo start point, l’end point e la mappa elaborata dal robot supervisore
- restituisce la mappa aggiornata che contiene i punti da visitare per andare dallo start point all’end point
All’interno vengono eseguite le seguenti operazioni:
- riceve la mappa dove ogni sottolista indica [Visitata, Ovest, Nord, Est, Sud]
<img src="https://user-images.githubusercontent.com/10675526/219769608-5fbb128f-3adc-4463-954b-fda4e1af16e6.png" width="500"/>
<br><br><br>
- vengono applicata una maschera pesi fittizi infiniti verso tutti i percorsi per indicare all’algoritmo che ancora non sono stati presi in esame
<img src="https://user-images.githubusercontent.com/10675526/219770235-fa000075-021b-47a2-bce9-92b97bb44c14.png" width="500" />
<br><br><br>
- viene creata la maze dove righe e colonne dispari indicano i muri fittizi che servono per potere identificare le celle attigue dalle quali non è possibile passare da una all’altra
<img src="https://user-images.githubusercontent.com/10675526/219771121-9a488ab2-242d-41bd-950f-071c1f8596e4.png" width="500" />
quindi per una mappa 4x4 la maze avrà una dimensione 7x7, mentre per una mappa 8x8 sarà mappata in una 15x15

- vengono inseriti gli ostacoli reali nella maze:
[[0, 0, 0, 0, 0, 0, 0],
[0, 1, 1, 1, 0, 1, 0],
[0, 1, 0, 1, 0, 1, 0],
[1, 1, 0, 1, 0, 1, 0],
[0, 1, 0, 0, 0, 1, 0],
[0, 1, 1, 1, 0, 1, 0],
[0, 0, 0, 0, 0, 0, 0]]

- l’algoritmo ASTAR calcola il percorso su quest’ultima lista di liste:
[[-1, -1, -1, -1, -1, -1, -1],
[-1, -1, -1, -1, -1, -1, -1],
[-1, -1, 0, -1, -1, -1, -1],
[-1, -1, 1, -1, -1, -1, -1],
[12, -1, 2, 3, 4, -1, -1],
[11, -1, -1, -1, 5, -1, -1],
[10, 9, 8, 7, 6, -1, -1]]
dove il percorso da seguire è dato dai numeri >= 0

- vengono scartati i punti contenenti i muri virtuali, si torna quindi ad una 4x4:
<img src="https://user-images.githubusercontent.com/10675526/219771992-ce088d67-c5dd-454b-9f29-344ea818a84d.png" width="500" />
viene ricomposta in una sequenza ordinata:
[ 
  [-1,-1,-1,-1], [-1,2,-1,-1], [8,3,4,-1], [7,6,5,-1] 
]

- viene restituita la grid_maze al controllore, aggironata e contenente come primo elemneto un numero che indica come valore il passaggio nel percorso tra il punto iniziale e finale :
[
  [2, 'X', 'X', 0, 0], [3, 0, 'X', 0, 1], [4, 0, 'X', 0, 0], ['inf', 0, 'X', 'X', 0],
  ['inf', 'X', 0, 1, 1], ['inf', 1, 1, 1, 0], [5, 1, 0, 1, 0], ['inf', 1, 0, 'X', 0],
  ['inf', 'X', 1, 1, 0], ['inf', 1, 0, 0, 1], [6, 0, 0, 1, 0], ['inf', 1, 0, 'X', 0],
  ['inf', 'X', 0, 0, 'X'], ['inf', 0, 1, 0, 'X'], [7, 0, 0, 0, 'X'], [8, 0, 0, 'X', 'X'] 
]


### Emitter – Receiver
Per inviare un pacchetto di dati ai potenziali ricevitori, la funzione wb_emitter_send aggiunge alla coda dell'emettitore un pacchetto di byte di dimensioni pari all'indirizzo indicato da data.
I pacchetti di dati in coda saranno inviati ai potenziali destinatari (e rimossi dalla coda dell'emettitore) alla velocità specificata dal campo baudRate del nodo Emitter.
La coda è considerata piena quando la somma dei byte di tutti i pacchetti attualmente in coda supera la dimensione del buffer specificata dal campo bufferSize. 
In particolare, con Python, la funzione send invia una stringa; in alternativa si possono inviare tipi di dati primitivi in questa stringa con il modulo struct.
Quindi, nel nostro caso, prevedendo lo scambio di informazioni tra i robot, anche, con:
- lista di liste
- array
si è reso necessario ripensare la struttura dati in un formato del tipo:
<img src="https://user-images.githubusercontent.com/10675526/219773560-0ff6a235-257b-4191-8af6-4e136c7a8df8.png" width="600" />
in modo schematico:
- Lato Emitter:
<img src="https://user-images.githubusercontent.com/10675526/219774448-29b1f794-0d2a-404f-980e-93fcf32fe928.png" width="600" />
- Lato Reicever:
<img src="https://user-images.githubusercontent.com/10675526/219775530-d1f35359-537b-4a3c-ba9c-c81340560d77.png" width="600" />

I messaggi scambiati avranno una struttura del tipo:
- Lato Supervisore
<img src="https://user-images.githubusercontent.com/10675526/219778561-c3ec8699-d1d4-4b36-8e3d-7a9ffdd90c5a.png" width="600" />
<br>
- Lato Operaio
<img src="https://user-images.githubusercontent.com/10675526/219779176-93afde2d-64bb-4019-80ee-fa40a8d294b4.png" width="600" />
<br><br>

###Conclusioni
Abbiamo raggiunto il nostro obiettivo, creare un ambiente simulato di lavoro dove diversi robot riescono a cooperare per portare avanti un compito comune: la cura della coltura.

È stato possibile realizzarlo grazie all’ideazione di uno scenario dove sono presenti robot specializzati in determinate attività:
- Robot supervisore
  - Si localizza e Mappa un ambiente sconosciuto
  - Definisce i punti del giardino dove intervenire
  - Contatta via radio i robot operai comunicando i punti di lavoro e il compito da eseguire
  - 
- Robot operaio
  - Riceve via radio l’ambiente di lavoro sul quale operare e il compito da eseguire
  - Definisce con l’algoritmo A-Star il percorso per arrivare sul punto di lavoro
  - Esegue il compito nel quale è specializzato
  - Contatta il robot supervisore per informarlo che ha esaurito il compito
  - Resta in attesa di ulteriori istruzioni.


