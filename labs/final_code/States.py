
class States:
    class Speed:
        state = 10
        speed = 0.3
        name = "SPEED"
    class Stop:
        state = -1
        speed = 0 
        name = "Stop"
    class Line:
        state = 0
        fast_speed = 0.16
        slow_speed = 0.16
        name = "Line"
    class Wall:
        state = 1
        straight_speed = 0.15
        turn_speed = 0.16
        fast = 0.17
        name = "wall"
    class Cone:
        state = 2
        fast_speed = 0.18
        slow_speed = 0.15
        name = "Cone"
    class Lane:
        state = 3
        fast_speed = 0.2
        slow_speed = 0.1
        name = "Lane"