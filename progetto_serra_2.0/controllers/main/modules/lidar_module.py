class LidarDevice:  #Lidar Sensor Class custom
    LIDAR_SENSOR = None
    rangeImage = []
    limit_lidar = []
    
    def __init__ (self, lidarDevice, timestep):
        
        self.LIDAR_SENSOR = lidarDevice
        self.LIDAR_SENSOR.enable(timestep)
        self.LIDAR_SENSOR.enablePointCloud()
        self.rangeImage = self.getRangeImage()
        
        #Soglie di tolleranza per il lidar index 0,1,2, -> left, center, right
        for i in range(3):
            self.limit_lidar.append(0.18)
        
        
    def getRangeImage(self):
        # recupera il rangeImage dell'ultima scansione fatta,
        # la prendiamo sul sensore per avere i valori sempre aggiornati
        return self.LIDAR_SENSOR.getRangeImage()
        
    def getRangeImageByIndex(self, index):
        #Get rangeImage
        return (self.LIDAR_SENSOR.getRangeImage())[index]
    
    def getLimitLidar(self):
        return self.limit_lidar
    
    def getLimitLidarById(self, index):
        return self.limit_lidar[index]

    def getLidarVector(self):
        rangeImage = self.getRangeImage() 
        lidarVector =[rangeImage[0], rangeImage[255], rangeImage[511]] 
        if lidarVector[0] > 1: 
            lidarVector[0] = 1 
        if lidarVector[1] > 1: 
            lidarVector[1] = 1 
        if lidarVector[2] > 1: 
            lidarVector[2] = 1 
        return lidarVector