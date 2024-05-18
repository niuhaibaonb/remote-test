
from plexe import INDEX, U, ACCELERATION, SPEED, POS_X, POS_Y, TIME, LENGTH


class VehicleData:
    """
    Class to store data of a vehicle. Fields can either be accessed directly
    (e.g., data.acceleration) or using a dictionary-like access (e.g.,
    data[ACCELERATION]) depending on convenience
    """
    def __init__(self, index=None, u=None, acceleration=None, speed=None,
                 pos_x=None, pos_y=None, time=None, length=None):
        self.index = index
        self.u = u
        self.acceleration = acceleration
        self.speed = speed
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.time = time
        self.length = length

    def __getitem__(self, item):
        if item == INDEX:
            return self.index
        elif item == U:
            return self.u
        elif item == ACCELERATION:
            return self.acceleration
        elif item == SPEED:
            return self.speed
        elif item == POS_X:
            return self.pos_x
        elif item == POS_Y:
            return self.pos_y
        elif item == TIME:
            return self.time
        elif item == LENGTH:
            return self.length
