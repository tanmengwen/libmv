from pylab import *
from numpy import *

#TEST(unscented_class_2d_rotated_scaled_in_y)
a = pi/8.;
R = array([[cos(a), sin(a)],
           [-sin(a),cos(a)]])
S = diag([1.0,2.0])
X = array([[0.,1.,-1.,0.,0.],
           [0.,0.,0.,1.,-1.]])

plot(X[0,:],X[1,:],'r.')
M=dot(R,S) # transform!
print 'M'
print M
print 'inv M'
print linalg.inv(M)
Y=dot(M,X)
plot(Y[0,:],Y[1,:],'b.')
YY = dot(M,M) # cov
print 'new covariance'
print YY

axis('equal')

print '================================='
#TEST(unscented_class_2d_rotated_scaled_in_x)
a = pi/8.;
R = array([[cos(a), sin(a)],
           [-sin(a),cos(a)]])
S = diag([3.0,1.0])
X = array([[0.,1.,-1.,0.,0.],
           [0.,0.,0.,1.,-1.]])
M=dot(R,S)
print 'M'
print M
print 'inv M'
print linalg.inv(M)
Y=dot(M,X)
YY = dot(M,M.T)
print 'cov yy stretched x'
print YY
