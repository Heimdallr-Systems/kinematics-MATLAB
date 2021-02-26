import bpy
import os
os.system("cls")

counts = []

for object in bpy.data.objects:
    print(f"{object.name}: {len(object.data.vertices)}")