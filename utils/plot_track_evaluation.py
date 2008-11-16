from pylab import *
from numpy import *
import sys

class AttributeDict(object):
    "Attribute accesses are changed into a dictionary lookup"
    def __init__(self, x):
        self.stuff = x
    def __getattr__(self, name):
        return self.stuff[name]

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

def fill_between(y1,y2, x=None, **kwargs):
    n = len(y1)
    if x is None:
        x = arange(n)
    fill(hstack((x,x[::-1])),
         hstack((y2,y1[::-1])),**kwargs)

if __name__ == '__main__':
    results_file = 'eval.json'
    if len(sys.argv[0]) == 2:
        results_file = sys.argv[1]
    results = AttributeDict(eval(file(results_file).read()))

    smin,smax = calculate_bounds_from_tracks(results.tracks)
    print smin
    print smax
    scene_size =  norm(smax-smin)
    scale = 1./scene_size
    print 'sz', scene_size

    # find maximum track
    longest_track = 0
    for ltrack in results.tracks:
        if len(ltrack) > longest_track:
            longest_track = len(ltrack)

    bins = zeros((100, longest_track), Float64)
    totals = zeros((longest_track,), Float64)
    totals2 = zeros((longest_track,), Float64)
    counts = ones((longest_track,), Float64)
    outlier_counts = ones((longest_track,), Float64)
    hold(True)
#    for ltrack in results.tracks[:500]:
    all_distances = []
    for ltrack in results.tracks:
        track = array(ltrack)
        if track.shape == (0,):
            print distances
            continue
        delta = track - track[0,:]
        delta *= scale
        distances = (delta**2).sum(1)**.5
        if max(distances) > 0.5:
            print delta - delta[0,:]
#        plot(distances, 'r-')
        factor = 0.003
        for i,d in enumerate(distances[1:]):
            bin = int(bins.shape[0]*d/factor)
            bin = min(bin, bins.shape[0]-1)
#            print 'i=',i,'bin=', bin
            bins[bin, i] += 1.0
            if d > 0.005:
                outlier_counts[i] += 1.0
            else:
                totals[i] += d
                totals2[i] += d**2
                counts[i] += 1.0
        all_distances.extend(list(distances))
#    savefig('track_errors.png')
#    show()
#    close()
#    pcolor(bins, shading='flat')
#    savefig('track_error_over_time_plot.png')
    avg = totals/counts
    var = totals2/counts - avg**2

#    figure()
#    title('robust std dev')
#    plot(var**.5, 'r-')
    stddev = var**.5

    figure()
    hold(True)
    TOP = avg+stddev
    BOT = avg-stddev
    fill_between(BOT,TOP, alpha=0.3, facecolor='g')
    plot(avg+stddev, 'g--', linewidth=1)
    plot(avg-stddev, 'g--', linewidth=1)
#    plot(avg, 'g-', linewidth=2)
    width_factor = 1.0
    napolean_top = avg + width_factor*avg[0]*counts/max(counts)
    napolean_bot = avg - width_factor*avg[0]*counts/max(counts)
    plot(avg, 'k-', linewidth=2)
    fill_between(napolean_bot,napolean_top, alpha=0.8, facecolor='g')
    title('mean')

#    figure()
#    title('robust variance')
#    plot(var, 'r-')
#
    figure()
    title('inliers and outliers')
    plot(counts-1, 'g-',label='inliers')
    plot(outlier_counts-1, 'r-',label='outliers')
    plot(counts+outlier_counts-2, 'k.-',label='total')
    legend()

    show()
#    figure()
#    hist(all_distances, 2*sqrt(len(all_distances)))

