from generatedata import *

# looking down positive z axis
# scene is metric and at x,y in 1,1
# at distance z=3
num_inliers = 30
num_outliers = 20
X = 2*(rand(4,num_inliers+num_outliers)-0.5)
X[2,:] += 3
X[3,:] = 1.0

T = rand(4,4) # now projectively transform the scene
P = zeros((3,4))
P[:3,:3] = eye(3)
K = array([[2000.,  0., 700],
           [0.,  2000., 700],
           [0.,    0.,   1.]])
P = dot(K, P)
P = dot(P,T)
P /= sqrt(sum(P**2))
X = dot(linalg.inv(T), X)
x = dot(P,X)
x /= x[2,:]
x = x[:2,:]

# add substantial gaussian noise (5% of image size) to last pixels
# to make some outliers, but ones that 'look' like they might be
# right
noise = randn(2,num_outliers)
noise *= 0.05 * c_[amax(x,1) - amin(x,1)]

x[:,num_inliers:] += noise

#from pylab import *
#plot(x[0,:],x[1,:],'r.')
#show()

export('x X P num_inliers') 
