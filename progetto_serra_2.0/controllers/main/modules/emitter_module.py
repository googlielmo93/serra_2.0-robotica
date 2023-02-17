"""emitter module."""

import modules.message_modules as msg_m

from controller import AnsiCodes

class EmitterDevice():   #emitter
    
    EMITTER_SENSOR = None
    MESSAGE = msg_m.Message()
    
    def __init__(self, emitterDevice):
        self.EMITTER_SENSOR = emitterDevice
        #setto il range di comunicazione
        self.EMITTER_SENSOR.setRange(5)
    
    # Il metodo send accetta come argomento una struct in python che viene
    # restituita dal metodo packaging() della classe Message
    def send(self, data):
        dataPack = self.MESSAGE.packaging(data)
        self.EMITTER_SENSOR.send(dataPack)
            
    
    def setChannel(self, channel=-1):
        if channel != -1:   #Se viene passato un canale specifico su cui comunicare
            self.EMITTER_SENSOR.setChannel(channel)
        else:   #Settiamo l'invio del messaggio in broadcast
            self.EMITTER_SENSOR.setChannel(self.EMITTER_SENSOR.CHANNEL_BROADCAST)
            
            
    def getChannel(self):
        return self.EMITTER_SENSOR.getChannel()
