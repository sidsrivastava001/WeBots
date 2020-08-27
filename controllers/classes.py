import turtle


class Polygon:
  def __init__(self, sides, name, size=100, color='black', line_thickness=2):
        self.sides = sides
        self.name = name
        self.size = size
        self.interior_angles = (self.sides - 2) * 180
        self.angle = self.interior_angles/self.sides
        self.color = color
        self.line_thickness = line_thickness

  def draw(self):
    turtle.color(self.color)
    turtle.pensize(self.line_thickness)
    for i in range(self.sides):
          turtle.forward(self.size)
          turtle.right(180-self.angle)
    



class Square(Polygon):
  def __init__(self, size=100, color='black', line_thickness=2):
    super().__init__(4, "Square", size, color, line_thickness)

  def draw(self):
    turtle.begin_fill()
    super().draw()
    turtle.end_fill()




square = Square(color="#123abc", size=200)

print(square.sides)
print(square.angle)

print(square.draw())

turtle.done()

# square = Polygon(4, 'Square', 200)
# pentagon = Polygon(5, 'Pentagon', 200)
# hexagon = Polygon(6, 'Hexagon', color='red', line_thickness=20)

# print(square.sides)
# print(square.name)
# print(square.interior_angles)
# print(square.angle)

# print(pentagon.name)
# print(pentagon.sides)

# hexagon.draw()