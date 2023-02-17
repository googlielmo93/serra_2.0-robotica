"""receiver module."""

import struct

from collections import deque
import modules.message_modules as msg_m

class ReceiverDevice():   #receiver
    
    RECEIVER_SENSOR = None
    MESSAGE = msg_m.Message()
    
    def __init__(self, receiverDevice, timestep):
        self.RECEIVER_SENSOR = receiverDevice
        self.RECEIVER_SENSOR.enable(timestep)

    
    def getPackets(self, numEl):
        messageQueue = deque()
        
        while self.RECEIVER_SENSOR.getQueueLength() > 0:
            #   struct{
            #       id_response = id_request
            #       mittente = getName oppure il canale associato
            #       destinatario = getName oppure il canale associato
            #       mappa_aggiornata = come vettore di lookup
            #       percorso = vettore vuoto (dipende dal destinatario), oppure 
            #       time_current = timestep o timestamp
            #   }
            
            packed_data = self.RECEIVER_SENSOR.getData()
            
            messageDecoded = self.MESSAGE.unpackaging(packed_data, numEl)
            
            messageQueue.append(messageDecoded)
            
            self.RECEIVER_SENSOR.nextPacket()
            
        return messageQueue

    
    def setChannel(self, channel=-1):
        if channel != -1:   #Se viene passato un canale specifico su cui comunicare
            self.RECEIVER_SENSOR.setChannel(channel)
        else:   #Settiamo l'invio del messaggio in broadcast
            self.RECEIVER_SENSOR.setChannel(self.RECEIVER_SENSOR.CHANNEL_BROADCAST)
            
            
    def getChannel(self):
        return self.RECEIVER_SENSOR.getChannel()
    
     