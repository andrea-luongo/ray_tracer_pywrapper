import rayTracerPyWrapper as m
from rayTracerPyWrapper import PyBindBVH as e
import numpy as np
import time

print(dir(m), dir(e))
print(int(e.SAH), int(e.HLBVH), int(e.Middle), int(e.EqualCounts))
v = [1, 2, 3]
p = e(v, e.EqualCounts, 100)

primitives = []
number_of_primitives = 65000

print("generating primitives")
start_time = time.time()
for i in range(number_of_primitives):
	v0 = (np.random.rand(3) - 0.5) * 100
	v1 = (np.random.rand(3) - 0.5) * 100
	v2 = (np.random.rand(3) - 0.5) * 100
	primitives.append(v0)
	primitives.append(v1)
	primitives.append(v2)
print("generating primitives", time.time() - start_time)
start_time = time.time()
flat_list = [item for sublist in primitives for item in sublist]
print("converting primitives", time.time() - start_time)
start_time = time.time()
p = e(flat_list, e.EqualCounts, 100)
print("BVH construction", time.time() - start_time)