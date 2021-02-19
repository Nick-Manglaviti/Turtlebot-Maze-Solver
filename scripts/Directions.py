#! /usr/bin/env python

class Directions(object):

    @staticmethod
    def add_90_right(value):
        result = value - 90
        while (result > 180) or (result < -180):
            if result > 180:
                result -= 360
            elif result < -180:
                result += 360
        if result == -180:
            result = 180
        return result

    @staticmethod
    def add_90_left(value):
        result = value + 90
        while (result > 180) or (result < -180):
            if result > 180:
                result -= 360
            elif result < -180:
                result += 360
        if result == -180:
            result = 180
        return result
    
    @staticmethod
    def add_180(value):
        result = value + 180
        while (result > 180) or (result < -180):
            if result > 180:
                result -= 360
            elif result < -180:
                result += 360
        if result == -180:
            result = 180
        return result

    @staticmethod
    def get_direction_mapping(starting_orientation):
        north = starting_orientation
        south = add_180(self._north)
        east = add_90_right(self._north)
        west = add_90_left(self._north)
        directions = [self._north, self._south, self._east, self._west]
        return directions

    

if __name__ == "__main__":
    print(Directions.add_180(0))
    print(Directions.add_180(90))
    print(Directions.add_180(180))
    print(Directions.add_180(-90))