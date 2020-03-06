import numpy as np
import scipy
import pickle as pkl
a = np.fromfile("test.pcl")
b = a.reshape(30328,3)
#points = np.array([[0, 0], [0, 1.1], [1, 0], [1, 1]])
#from scipy.spatial import Delaunay
#tri = Delaunay(b)
print("DONE")
"""
plt.triplot(b[:,0],b[:,1], tri.simplices.copy())
plt.plot(b[:,0], b[:,1], 'o')

plt.show()
"""
"""
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

# The triangles in parameter space determine which x, y, z points are
# connected by an edge
#ax.plot_trisurf(x, y, z, triangles=tri.triangles, cmap=plt.cm.Spectral)
ax.plot_trisurf(tri[:,0], tri[:,1], tri[:,2], triangles=tri.simplices, cmap=plt.cm.Spectral)


plt.show()
"""


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.tri as mtri
from scipy.spatial import Delaunay

# u, v are parameterisation variables
u = np.array(b[:,0])
v = np.array(b[:,1])

x = u
y = v
z = np.array(b[:,2])

# Triangulate parameter space to determine the triangles
#tri = mtri.Triangulation(u, v)
tri = Delaunay(np.array([u,v]).T)
"""
print 'polyhedron(faces = ['
#for vert in tri.triangles:
for vert in tri.simplices:
    print '[%d,%d,%d],' % (vert[0],vert[1],vert[2]),
print '], points = ['
for i in range(x.shape[0]):
    print '[%f,%f,%f],' % (x[i], y[i], z[i]),
print ']);'
"""

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection='3d')

# The triangles in parameter space determine which x, y, z points are
# connected by an edge
#ax.plot_trisurf(x, y, z, triangles=tri.triangles, cmap=plt.cm.Spectral)
ax.plot_trisurf(x, y, z, triangles=tri.simplices, cmap=plt.cm.Spectral)
ax.set_xlabel("X")
ax.set_ylabel("Y")



plt.show()
#plt.savefig("A",format="SVG")
