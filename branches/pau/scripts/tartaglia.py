from numpy import *

t = [array([1])]
for i in xrange(35):
	n = zeros(t[i].shape[0]+1)
	n[1:] += t[i]
	n[:-1] += t[i]
	t.append(n)

for i in xrange(0,len(t),2):
	l = t[i]/sum(t[i])

	print 'static float weights%d[%d] = {'%(i+1,i+1),

	for x in l[:-1]:
		print x,',',
	print l[-1],

	print '};'
	
