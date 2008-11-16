from generatedata import *
n = 10
X = rand(4,n)
T = rand(4,4)
Xp = dot(T,X) * rand(1,n) # also scales each column randomly
Xpp = dot(T,X) # no scaling
T /= sqrt((T**2).sum())
export('X Xp Xpp T n')
