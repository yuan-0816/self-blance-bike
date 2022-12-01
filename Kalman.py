# Kalman Filters by YUAN
class Kalman():
    def __init__(self):
        self.Q_angle = 0.001
        self.Q_gyro = 0.003
        self.R_angle = 0.5
        self.Q_bias = 0.0
        self.angle = 0.0
        self.K_0 = 0.0
        self.K_1 = 0.0
        self.P = [[1.0, 0.0], [0.0, 1.0]]

    def getAngle(self, newAngle, newGyro, dt):
        
        self.angle = self.angle - (self.Q_bias * dt) + (newGyro * dt)
        
        self.P[0][0] = self.P[0][0] - (self.P[0][1] + self.P[1][0]) * dt + (self.P[1][1] * dt * dt) + self.Q_angle
        self.P[0][1] = self.P[0][1] - (self.P[1][1] * dt)
        self.P[1][0] = self.P[1][0] - (self.P[1][1] * dt)
        self.P[1][1] = self.P[1][1] + self.Q_gyro
        
        self.K_0 = self.P[0][0]/(self.P[0][0]+self.R_angle)
        self.K_1 = self.P[1][0]/(self.P[0][0]+self.R_angle)
        
        self.angle = self.angle + self.K_0 * (newAngle - self.angle)
        self.Q_bias = self.Q_bias + self.K_1 * (newAngle - self.angle)
        
        self.P[0][0] = self.P[0][0] - self.K_0 * self.P[0][0]
        self.P[0][1] = self.P[0][1] - self.K_0 * self.P[0][1]
        self.P[1][0] = self.P[1][0] - self.K_1 * self.P[0][0]
        self.P[1][1] = self.P[1][1] - self.K_1 * self.P[0][1]
        
        return self.angle
        
    def setAngle(self, angle):
        self.angle = angle
        
    def setQ_angle(self, Q_angle):
        self.Q_angle = Q_angle

    def setQ_bias(self, Q_bias):
        self.Q_bias = Q_bias

    def setR_angle(self, R_angle):
        self.R_angle = R_angle

    def getR_angle(self):
        return self.R_angle

    def getQ_angle(self):
        return self.Q_angle

    def getQ_bias(self):
        return self.Q_bias

    def  getR_angle(self):
        return self.R_angle
        
    
