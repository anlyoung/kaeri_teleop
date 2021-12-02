

import stl
from stl import mesh
import numpy 

your_mesh = mesh.Mesh.from_file('baxter_foam_cutter.STL')

your_mesh.save('foam_cutter.STL',mode=stl.Mode.BINARY)
