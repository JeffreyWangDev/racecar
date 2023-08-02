from typing import enum

class States:
    class Line:
        state = 0
        fast_speed = 0.2
        slow_speed = 0.1
    class Wall:
        state = 1
        fast_speed = 0.2
        slow_speed = 0.1
    class Cone:
        state = 2
        fast_speed = 0.2
        slow_speed = 0.1