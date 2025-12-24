import numpy as np
import matplotlib.pyplot as plt

class Robot:
    def __init__(self):
        self.state = np.array([0.0,0.0,0.0])
        self.history = []
        self.save_history()

    def save_history(self):
        self.history.append(self.state[:2].copy())

    def move(self, speed, seconds):
        theta=self.state[2]
        dx=speed*seconds*np.cos(theta)
        dy=speed*seconds*np.sin(theta)

        self.state[0] += dx
        self.state[1] += dy
        self.save_history()
    def turn(self, radians):
        self.state[2] += radians
        self.state[2] = (self.state[2] +np.pi) % (2*np.pi)-np.pi
        self.save_history()

bot = Robot()

print("Iniciando mision")
lados=4
for i in range(lados):
    bot.move(speed=1.0, seconds=2.0)
    bot.turn(np.pi/2 +0.1)

print("generacion de grafica")
path=np.array(bot.history)

plt.figure(figsize=(8,8))
plt.plot(path[:,0], path[:,1], '-o', label='Trayectoria Robot')

plt.grid(True)
plt.axis('equal')
plt.title("Simulacion de Odometria")
plt.xlabel("X [metros]")
plt.ylabel("Y [metros]")
plt.legend()
plt.show()


