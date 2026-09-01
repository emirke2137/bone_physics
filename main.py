import bpy
from mathutils import Vector
##importing own modules



register = bpy.data.texts["handler.py"].as_module().register
##-----------------------

enabled = True

if __name__ == '__main__':
      
    #load_rig_data()
    #if enabled
    register()
    #else
    #unregister
    
    
    
    