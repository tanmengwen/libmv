from pylab import *
from numpy import *
import sys

results_file = 'eval.json'
if len(sys.argv) == 2:
    results_file = sys.argv[1]

# hack alert! heh
class AttributeDict(object):
    "Attribute accesses are changed into a dictionary lookup"
    def __init__(self, x):
        self.stuff = x
    def __getattr__(self, name):
        if name in self.stuff:
            return self.stuff[name]
        return self.stuff.__getattribute__(name)
results = AttributeDict(eval(file(results_file).read()))

error_thresh = results.error_thresh

def calculate_bounds_from_tracks(tracks):
    # calculate scene bounds
    smin = zeros(3,Float64)
    smax = zeros(3,Float64)
    for track in tracks:
        for thistrack in track:
            pt = array(thistrack)
            smin = amin(vstack((smin,pt)),0)
            assert(len(smin)==3)
            smax = amax(vstack((smax,pt)),0)
    return smin,smax

#for i in range(results.tracks):
#    if len(results.tracks[i]) == 0:
#        del results.tracks[i]
#

smin,smax = calculate_bounds_from_tracks(results.tracks)
print smin
print smax
scene_size =  norm(smax-smin)
scale = 1./scene_size
print 'sz=', scene_size
print 'scale=', scale
#print 'all_residulas=',results.all_residuals 
results.all_residuals = array(results.all_residuals)

# normalize residuals by scene size
results.all_residuals *= scale
error_thresh *= scale

print 'ntracks=', len(results.tracks)

################################################################################
import pylab
pylab.rcParams.update({'xtick.labelsize':8,'ytick.labelsize':8})
figure()
hold(True)
subplot(111)
title('x vs y of tracked points ground truth (no reconstruction)')
axis('off')
all_mag_errors = []
for i in range(5):
    for j in range(5):
        subplot(5,5,5*i+j+1)
#        plot(array(results.tracks[5*i+j])[0])
        arr = array(results.tracks[5*i+j]).T.copy()
        # subtract off the mean
        arr -= c_[sum(arr,1)/float(arr.shape[1])]
        arr *= scale # account for scaling
#        arr *= 10000
#        plot(arr[0,:], arr[1,:], 'r.');
#        plot(arr[0,:], arr[2,:], 'g.');
#        plot(arr[1,:], arr[2,:], 'b.');
#        plot([0], [0], 'bs', mfc=None);
        mags = sum(arr**2, 0)**.5
        all_mag_errors.append(mags)
        hist(mags)
#        axis('equal')
#        axis('scaled')
#        imshow(cov(array(results.tracks[5*i+j]).T),interpolation='nearest')
#        axis('off')
        if not (i==4 and j ==0):
#            xticks([])
            yticks([])
            zz=get(gca(), 'xticklabels')
            setp(zz, 'rotation', 'vertical')
        else:
#            y = yticks()
#            setp(y, fontsize=7)
#            xlabel('normalized scene distance x', fontsize=7)
#            ylabel('normalized scene distance y', fontsize=7)
            zz=get(gca(), 'xticklabels')
            setp(zz, 'rotation', 'vertical')
savefig('track_resids_hists.png')

################################################################################
figure()
all_mag_errors = []

for i in range(len(results.tracks)):
    if len(results.tracks[i]) == 0:
        continue
    arr = array(results.tracks[i]).T.copy()
    # subtract off the mean
    arr -= c_[sum(arr,1)/float(arr.shape[1])]
    arr *= scale # account for scaling
#   arr *= 10000
    mags = sum(arr**2, 0)**.5
    all_mag_errors.extend(list(mags))
n = 1.5
m=1
allmags = array(all_mag_errors)
allmags = allmags[allmags < n*error_thresh]
hist(allmags, (m*len(allmags)**.5), lw=0, alpha=0.5, label='distance from mean tracked point')
print array(results.all_residuals).shape
print 'len(res.ar)=',len(results.all_residuals)
print results.keys()
norms = sqrt(sum(results.all_residuals**2, 0))
norms = norms[norms < n*error_thresh]
hist(norms, (m*len(norms)**.5), lw=0, fc='red', alpha=0.5, label='distance from reconstructed point')
xlabel('normalized scene distance (1.0 = diameter of scene)')
title('comparison of raw tracking errors to final reconstruction errors')
#legend()
yticks([])
savefig('track_resids_hist.png')

################################################################################
import pylab
from matplotlib.font_manager import FontProperties
pylab.rcParams.update({'xtick.labelsize':8,'ytick.labelsize':8})
figure()
hold(True)
subplot(111)
title('x vs y of tracked points ground truth (no reconstruction)')
axis('off')
for i in range(5):
    for j in range(5):
        subplot(5,5,5*i+j+1)
        arr = array(results.tracks[5*i+j]).T.copy()
        # subtract off the mean
        arr -= c_[sum(arr,1)/float(arr.shape[1])]
        arr *= scale # scale scene diameter to 1.0
        arr *= 10000 # make numbers sane
        plot(arr[0,:], arr[1,:], 'r.', label='x vs y');
        plot(arr[0,:], arr[2,:], 'g.', label='x vs z');
        plot(arr[1,:], arr[2,:], 'b.', label='y vs z');
        plot([0], [0], 'bs', mfc=None);
        axis('equal')
        axis([-3,3,-3,3])
        if not (i==4 and j ==0):
            xticks([])
            yticks([])
        else:
            xlabel('distance * 10000', fontsize=7)
            ylabel('distance * 10000', fontsize=7)
            zz=get(gca(), 'xticklabels')
            setp(zz, 'rotation', 'vertical')
            legend(prop=FontProperties(size=7))
savefig('track_resids.png')

################################################################################
figure()
for i in range(5):
    for j in range(5):
        subplot(5,5,5*i+j+1)
        arr = array(results.tracks[5*i+j]).T.copy()
        arr *= scale # account for scaling
        imshow(cov(arr), interpolation='nearest')
        gray()
        axis('off')
savefig('track_resids_covs.png')

################################################################################
figure()
maxy = 0
labels = 'xyz'
n=2
for i in range(3):
    r = results.all_residuals[i,:]
    r = r[abs(r) < n*error_thresh]
    nbins = .5*sqrt(len(r))
    hist(r,nbins, normed=True)
    a = axis()
    maxy = max(maxy, a[3])
clf()
for i in range(3):
    subplot(3,1,i+1)
    r = results.all_residuals[i,:]
    r = r[abs(r) < n*error_thresh]
    nbins = .5*sqrt(len(r))
    hist(r,nbins, normed=True)
    a = axis()
    axis([-n*error_thresh, n*error_thresh, 0, 1.2*maxy])
    ylabel('residual %s error' % labels[i])
savefig("resids_hist_xyz.png");

################################################################################
figure()
al = 0.05
sz = 40
scatter(results.all_residuals[0,:], results.all_residuals[1,:], sz, 'r', label='eX vs eY', alpha=al, faceted=False)
scatter(results.all_residuals[0,:], results.all_residuals[2,:], sz, 'g', label='eX vs eZ', alpha=al, faceted=False)
scatter(results.all_residuals[1,:], results.all_residuals[2,:], sz, 'b', label='eY vs eZ', alpha=al, faceted=False)
axis('equal')
title('residual error')
legend()
savefig("resids_plot_xyz.png");

################################################################################
figure()
hold(True)
al = 0.05
sz = 40
scatter(results.all_residuals[0,:], results.all_residuals[1,:], sz, 'r', label='eX vs eY', alpha=al, faceted=False)
scatter(results.all_residuals[0,:], results.all_residuals[2,:], sz, 'g', label='eX vs eZ', alpha=al, faceted=False)
scatter(results.all_residuals[1,:], results.all_residuals[2,:], sz, 'b', label='eY vs eZ', alpha=al, faceted=False)
axis('equal')
a = axis()
n = 2
axis([-n*error_thresh, n*error_thresh, -n*error_thresh, n*error_thresh])
title('residual error (zoom in to 3*error_thresh)')
legend()
savefig("resids_plot_zoom_xyz.png");
#savefig("resids_plot_zoom_xyz.pdf");

################################################################################
def vline(x):
    a=axis()
    plot([x,x],a[0:2],'k-')

figure()
norms = sqrt(sum(results.all_residuals**2, 0))
n = 3
norms = norms[norms < n*error_thresh]
hist(norms, 0.5*sqrt(len(norms)), normed=1);
title('histogram of magnitude of error in 3space (dist in R3)')
vline(error_thresh)
savefig("resids_mag.png");
