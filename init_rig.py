import bpy
from mathutils import Vector

##developmen area
data = bpy.data.texts["data.py"].as_module()
Point = data.Point
Constraint = data.Constraint

##-----------------------

armature = bpy.data.objects["Armature"]
control_bones = [
    bone for bone in armature.pose.bones
    if bone.parent is None
]
roots = [Point((armature.matrix_world @ bone.tail).copy(),
                            Vector((0,0,0)),
                            bone.name,
                            )
        for bone in control_bones]
constraints=[]
points=[]
def step(bone,parent):
    if bone.children:

        for c_bone in bone.children:
            #create point
            point = Point(
                            (armature.matrix_world @ c_bone.tail).copy(),
                            Vector((0,0,0)),
                            c_bone.name,
                            parent,
                            1,
                            (armature.matrix_world.to_3x3() @ bone.x_axis).normalized()
                            #armature.matrix_world -> obj
                            #to_3x3() -> only part of the mattrix that has rotation data
                            #to_2x2() -> only z rotation
                            #.x_axis -> rotaation and location in x axisg
                            ) 
            #constrain to parent
            con = Constraint(parent, point, c_bone.length)
            constraints.append(con)
            points.append(point)
            print(c_bone.name, "connects to ", bone.name)
            step(c_bone,point)    



def init_rig():
    for i in range(len(roots)):
        points.append(roots[i])
        step(control_bones[i],roots[i])
    
    for p in points:
        print(p.name)
    return constraints,points,roots, control_bones
