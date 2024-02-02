from rtree import index
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


#Rtree is a ctypes Python wrapper of libspatialindex that provides a number of advanced spatial 
#indexing features for the spatially curious Python user. These features include:

#COnstriuct an instance 
idx = index.Index()
#fter instantiating the index, create a bounding box that we can insert into the index:
left, bottom, right, top = (0.0, 0.0, 1.0, 1.0)
print("left: ", left)
print("bottom: ", bottom)
print("right: ", right)
print("top: ", top)

#the order of the coordinates depend by tyhe flag interleaved
idx.insert(0, (left, bottom, right, top))

#Indices that are contained inside the bounding 
print("indices contained inside the BB: ", list(idx.intersection((0.5, 0.5, 0.8, 0.8))))

#Return the first closer index of the BB to the new inserted BB.
#Return [0, 1], the index 0 of the new BB is closer to the index 1 of the original BB
idx.insert(1, (left, bottom, right, top))
print("indices nearest to BB: ",list(idx.nearest((0.0000001, 0.0000001, 2.0, 2.0), 1)))


from matplotlib.patches import Ellipse

plt.figure()
ax = plt.gca()

ellipse = Ellipse(xy=(157.18, 68.4705), width=0.036, height=0.012, 
                        edgecolor='r', fc='None', lw=2)
print("ellipse: ", ellipse)
ax.add_patch(ellipse)

