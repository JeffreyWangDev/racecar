
class States:
    class Line:
        state = 0
        fast_speed = 0.2
        slow_speed = 0.1
        name = "Line"
    class Wall:
        state = 1
        straight_speed = 0.2
        turn_speed = 0.155
        name = "wall"
    class Cone:
        state = 2
        fast_speed = 0.18
        slow_speed = 0.1
        name = "Cone"
    class Lane:
        state = 3
        fast_speed = 0.2
        slow_speed = 0.1
        name = "Lane"