"""message module."""

import random
import string
import struct
import ctypes

from controller import AnsiCodes

class Message():   #message
        
    # creo un packet message considera la seguente struttura dati:
    #
    #   struct{
    #       id_response = id_request
    #       mittente = getName oppure il canale associato
    #       destinatario = getName oppure il canale associato
    #       mappa_aggiornata = come vettore di lookup
    #       operazione = testo per definire la logica
    #       time_current = timestep o timestamp
    #   }
    #
    # La struct sopra verrà implementata in python con un dizionario come
    # segue partendo da una lista passata come argomento alla funzione:
    #
        # packet = {
        #       "id_message" : ...,
        #       "mittente" : ...,
        #       "destinatario" : ...,
        #       "operazione" : [...],
        #       "goalPos" : ...,
        #       "time_send" : ...
        #       "mappa" : [...],
        # }

    
        
        
    #genera una stringa alfanumerica di 16 caratteri    
    def randomString(self):
        return ''.join(random.choices(string.ascii_letters + string.digits, k=16))
    
        
    # come argomento accetta una lista dei valori sopra definiti.
    # return un dizionario nel formato sopra definito
    def packaging(self, packet):
        
        # inserisce l'ID del messaggio generato randomicamente e
        # sempre di 16 caratteri alfanumerici
        packet.insert(0, self.randomString())

        # flatto la mappa
        flatMap = self.flatten_list(packet[6])

        packed_data = struct.pack(
            '<16s 20s 20s 20s i f %uQ' % len(flatMap),  #format string
            packet[0].encode('utf-8', 'replace'),  #"id_message"
            packet[1].encode('utf-8', 'replace'),  #"mittente"
            packet[2].encode('utf-8', 'replace'),  #"destinatario"
            packet[3].encode('utf-8', 'replace'),  #"operazione"
            packet[4],                             #"goalPos"
            packet[5],                             #"time_send"
            *flatMap                               #"mappa"
        )
        
        return packed_data
    
    
    
    # come argomento accetta una lista dei valori sopra definiti.
    # return un dizionario nel formato sopra definito
    def unpackaging(self, packet, numEl):
        
        unpacked_data = struct.unpack(
            '<16s 20s 20s 20s i f %uQ' % (numEl),
            packet
        )
        
        data = list(unpacked_data)
        dataList = []
        
        
        # append stringhe decodificate in utf-8
        for i in range(4):
            dataList.append(data[i].decode('utf-8', 'replace').rstrip('\x00'))
            
        # append goal position
        dataList.append(data[4])
        # append time message
        dataList.append(data[5])
        
        # append map unflat
        dataList.append(self.unflat(data[6:], 5))
        
        return dataList

    
    def flatten_list(self, mapList): 
        flat_list = [] 
        for element in mapList: 
            if type(element) is list: 
                for item in element: 
                    flat_list.append(item) 
            else: 
                flat_list.append(element) 
        return flat_list 
    
    
    def unflat(self, listVal, n):
        lista = []
        for i in range(0, len(listVal), n):
            lista.append(listVal[i:i+n])
        return lista
                