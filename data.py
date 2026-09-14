from mathutils import Matrix
class Point:

    def __init__(self, pos, v, name, prev_point=None, weight=0, orientation=None,rest=None):
        self.pos = pos
        self.prev_pos = pos
        self.velocity = v
        self.weight = weight
        self.name = name
        self.prev_point = prev_point
        self.orientation = orientation
        self.rest_position = rest
        self.colision_radius = 0.002


class Constraint:

    def __init__(self,point_a, point_b, distance, compliance=1):
        self.point_a = point_a
        self.point_b = point_b
        self.distance = distance
        self.compliance = compliance
        

class CollisionConstraint:
    def __init__(self,point,location,normal):
        self.point = point
        self.location = location
        self.normal = normal
        
