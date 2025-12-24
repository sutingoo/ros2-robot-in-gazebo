import numpy as np

class Robot:
    def __init__(self, name):
        self.name = name
        self.state = np.array([0.0,0.0,0.0])

    def move(self,speed,seconds):
        d =speed*seconds
        theta=self.state[2]
        delta = np.array([
            d*np.cos(theta),
            d*np.sin(theta),
            0.0
        ])
        self.state += delta
        print(f"Movimiento {d}m. Estado Actual: {self.state}")

    def turn(self, radians):
        self.state[2] += radians
        self.state[2] = (self.state[2] + np.pi) % (2*np.pi)-np.pi
        print(f"Giro:{radians}rad. Orientacion: {self.state[2]:.2f}rad")

if __name__=="__main__":
    bot = Robot("Wall-E")
    bot.move(speed=1.0, seconds=5.0)
    bot.turn(np.pi/2)
    bot.move(speed=1.0, seconds=3.0)
    print(f"Posicion final del vector: {bot.state}")
 
