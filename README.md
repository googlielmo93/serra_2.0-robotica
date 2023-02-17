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



