# Dji TelloPy Pose estimation and FaceTracking


![](/docs/drone_gif.gif)
 
L’obiettivo di questo progetto è stato quello di sviluppare il software necessario per controllare gli spostamenti del drone DJITello mediante il riconoscimento facciale e il riconoscimento delle pose della persona inquadrata dalla camera del drone. Si è realizzato, in particolare, un software che consenta al drone di mantenere l’inquadratura dei soggetti durante il moto e che permetta inoltre di comandare gli spostamenti del drone in base a determinate posizioni delle braccia che il drone è in grado di riconoscere

# Pose estimation
L’utilizzo della pose estimation permette di stimare la posizione della faccia anche in condizioni non ottimali, per esempio in caso di buio, o se questa è leggermente coperta. Infatti sfruttando lastima della posizione del naso all’interno del frame, si è in grado di avere una posizione target acui fare riferimento per il face tracking.  Per aumentare le performance del facetracking rispettoalle possibili oscillazioni e alla velocità di risposta, si è deciso di utilizzare un controllore PID,che si è dovuto tarare opportunamente.

# Libreria per la Pose Estimation
La libreria utilizzata per effettuare la pose recognition è quella presente al seguente link: https://github.com/ZheC/tf-pose-estimation , a cui so-no state apportate alcune modifiche per poter essere meglio sfruttata per il lavoro svolto. La libreria sfrutta Tensorflow 1.4.1+ e OpenCV3 per la stima e le tecniche di Computer Vision. Il training è stato fatto con diversi modelli in particolare i modelli COCO cmu, dsconv, mobilenet_accurate,mobilenet, mobilenet_fast.

# Controllo

 Il controllo del drone avviene mediante una macchina a stati finiti
 
<img src="/docs/stati.PNG" width=500>
 

0.  in questo stato il drone è acceso ed è in attesa del segnale di avvio che viene dato coprendoe scoprendo ad intermittenza la camera
1.   In questo stato il drone entra nello stato di facetracking e pose recognition dove il dronesegue la faccia della persone di fronte e cerca di riconoscere la posa corrispondente;
2.   In questo stato il drone, quando non trova una persona di fronte a sé, entra in uno stato diricerca dove controlla attorno per trovare la persona di cui fare pose recognition;
3.   In  questo  stato  il  drone  esegue  la  mossa  impostata  per  poi  tornare  nello  stato  difacetracking;4.  infine questo stato è quello in cui il drone esegue la procedura di terminazione e termina. 


# Configurazione ambiente 

  * Creazione  dell'env
      - virtualenv --python=/usr/bin/python3.7m DjiTelloPose
  * Clonare la repository all'interlo dell'env
  * Eseguire il source 
      - source bin/activate
  * Installare i requirements
      - pip3 install -r requirements.txt  


# Avvio del programma

  * eseguire il source
      - source bin/activate

  * Avviare il programma tramite il seguente comando
      ./run.sh

