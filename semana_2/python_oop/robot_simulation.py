import math

class Robot:
    def __init__(self, name, x=0.0, y=0.0):
        self.name = name
        self.x = x
        self.y = y
        self.angle = 0.0
        print(f"Sistema: Robot {self.name} inicializado en ({self.x},{self.y})")

    def move(self, distance):
        self.x += distance* math.cos(self.angle)
        self.y += distance*math.cos(self.angle)
        print(f"Moviendo {distance}m. Nueva posicion: ({self.x:.2f},{self.y:.2f})")

    def turn(self, radians):

        self.angle += radians
        print(f"Giro: {radians}rad. Nuevo angulo: ({self.angle:.2f}rad)")

    def get_position(self):
        return(self.x, self.y)

if __name__ == "__main__":
    my_robot = Robot("R2D2")

    my_robot.move(2.0)
    my_robot.turn(math.pi / 2)
    my_robot.move(2)

    final_pos = my_robot.get_position()
    print(f"Posicion final {final_pos}")
