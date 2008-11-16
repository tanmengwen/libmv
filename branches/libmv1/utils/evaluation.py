from pylab import *
from numpy import *

#### These are for the extraction code
def reduce_into_dictionary(dictionary, name, new_vector, reducer):
    if name in dictionary:
        replacement = reducer(new_vector, dictionary[name])
    else:
        replacement = new_vector
    dictionary[name] = replacement

def reduce_min_vector(a,b):
    return amin(vstack((a,b)),0)
def reduce_max_vector(a,b):
    return amin(vstack((a,b)),0)
def update_min_vector(dictionary, name, new_vector):
    reduce_into_dictionary(dictionary, name, new_vector, reduce_min_vector)
def update_max_vector(dictionary, name, new_vector):
    reduce_into_dictionary(dictionary, name, new_vector, reduce_max_vector)
def update_max(dictionary, name, x):
    reduce_into_dictionary(dictionary, name, x, max)
def update_min(dictionary, name, x):
    print 'UPDATE:',name,'old=',x
    reduce_into_dictionary(dictionary, name, x, min)
    print '==>UPDATE:',name,'new=',dictionary[name]

class MetricExtractor(object):
    def __init__(self, Metric, evals):
        self.evals = evals
        self.Metric = Metric
        self.global_info = dict()

    def extract_metrics(self):
        self.metrics = [self.Metric(eval_data, self.global_info, self)
                        for eval_data in self.evals]

    def make_figure(self):
        figure()

    def finish_figure(self):
        show()

    def plot(self):
        self.extract_metrics()
        self.make_figure()
        for m in self.metrics:
            m.calculate()
        for m in self.metrics:
            m.plot()
        self.finish_figure()

#### End general metric extractor code

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

## Distance from first track

class DistanceFromFirstTrackExtractor(MetricExtractor):
    def __init__(self, evals):
        MetricExtractor.__init__(self, DistanceFromFirstTrack, evals)
    def make_figure(self):
        figure(figsize=(8,6))
    def finish_figure(self):
        xlabel('Track position [frames]')
        ylabel('3D error [scene diameter]')
        x0,x1,y0,y1 = axis()
        axis([x0,x1,0,y1])
        legend(loc='upper left')
        show()

class DistanceFromFirstTrack(object):
    def __init__(self, eval_data, global_info, extractor):
        self.extractor = extractor
        self.eval_data = eval_data
        self.info = global_info

    def calculate(self):
        eval_data = self.eval_data
        smin,smax = calculate_bounds_from_tracks(self.eval_data['tracks'])
        update_max_vector(self.info, 'smax', smax)
        update_min_vector(self.info, 'smin', smin)
        self.info['scale'] = 1.0/norm(self.info['smax']-self.info['smin'])

        # find maximum track
        longest_track = 0
        for ltrack in eval_data['tracks']:
            if len(ltrack) > longest_track:
                longest_track = len(ltrack)
        update_max(self.info, 'longest_track', longest_track)
        self.longest_track = longest_track

        totals = zeros((longest_track,), Float64)
        totals2 = zeros((longest_track,), Float64)
        counts = ones((longest_track,), Float64)
        outlier_counts = ones((longest_track,), Float64)
        for ltrack in eval_data['tracks']:
            track = array(ltrack)
            if track.shape == (0,):
#                print distances
                continue
            delta = track - track[0,:]
            distances = (delta**2).sum(1)**.5
            for i,d in enumerate(distances[1:]):
                if d > 0.350:
                    outlier_counts[i] += 1.0
                else:
                    totals[i] += d
                    totals2[i] += d**2
                    counts[i] += 1.0
        avg = totals/counts
        var = totals2/counts - avg**2
        print 'avg in calc', avg[:10]
        update_min(self.info, 'initial_width', avg[0])
        update_max(self.info, 'initial_count', amax(counts))
        self.avg = avg
        self.stddev = var**.5
        self.counts = counts
        self.outlier_counts = outlier_counts

    def plot(self):
        scale = self.info['scale']
        color = self.eval_data['color']
        avg = scale*self.avg
        stddev = scale*self.stddev
        counts = self.counts
        outlier_counts = self.outlier_counts
        TOP = avg+stddev
        BOT = avg-stddev
#        fill_between(BOT,TOP, alpha=0.3, facecolor=color, edgecolor=color)
#        plot(avg+stddev, color+'--', linewidth=1)
#        plot(avg-stddev, color+'--', linewidth=1)
#        width_factor = 1.0
#        napolean_top = avg + width_factor*avg[0]*counts/max(counts)
#        napolean_bot = avg - width_factor*avg[0]*counts/max(counts)
#        fill_between(napolean_bot, napolean_top,
#                     alpha=0.8, facecolor=color, edgecolor=color,
#                     label=self.eval_data['label'])
        min_avg = scale * self.info['initial_width']
        max_count = self.info['initial_count']
        napolean_top = avg + min_avg*counts/max_count
        napolean_bot = avg - min_avg*counts/max_count
        fill_between(napolean_bot, napolean_top,
                     alpha=0.5, facecolor=color, edgecolor='w',
                     linewidth=0.0,
                     label=self.eval_data['label'])
        plot(avg, color+'-', linewidth=1)
#        plot(avg, color+'-', linewidth=2, label=self.eval_data['label'])

def stretch_to(y, size):
    import scipy.interpolate
    x_actual = arange(0.0, 1.0, 1.0/len(y))
    x_new = arange(0.0, 1.0, 1.0/size)
    f=scipy.interpolate.interp1d(arange(0.0, 1.0, 1.0/len(y)),y)
    return f(x_new)

class AllMagErrorsExtractor(MetricExtractor):
    def __init__(self, evals):
        MetricExtractor.__init__(self, AllMagErrors, evals)
    def make_figure(self):
        figure(figsize=(8,6))
    def finish_figure(self):
        # assume all ground truth for tracks same for now
        tracks = self.metrics[0].eval_data['tracks']
        all_mag_errors = []
        for i in range(len(tracks)):
            xxx = tracks[i]
            if len(xxx) == 0:
                continue
            arr = array(xxx).T.copy()
            # subtract off the mean
            arr -= c_[sum(arr,1)/float(arr.shape[1])]
            mags = sum(arr**2, 0)**.5
            all_mag_errors.extend(list(mags))
        all_mag_errors.sort()
        error_thresh = all_mag_errors[len(all_mag_errors)/2]*4
        all_mag_errors = array(all_mag_errors)
        all_mag_errors = all_mag_errors[all_mag_errors < error_thresh]
        all_mag_errors *= self.global_info['scale']

#        hist(all_mag_errors, sqrt(len(all_mag_errors)),
#            normed=True,
#             label='ground truth',
#             alpha=0.5,
#             lw=0,
#             fc='b',
#             )

        all_mag_errors.sort()
        all_mag_errors = stretch_to(all_mag_errors, 1000)
        print 'ground truth', all_mag_errors[800]
        x_new = arange(0.0, 1.0, 1.0/1000)
        plot(x_new,all_mag_errors, linewidth=3,label='distance from average ground truth')
        ylabel('3D error [scene diameter]')
        xlabel('Fraction')
#        x0,x1,y0,y1 = axis()
#        axis([x0,x1,y0,0.024])
        legend(loc='upper left')
        title('Sorted reconstruction error (R^3 distance)')

#        x0,x1,y0,y1 = axis()
#        axis([x0,self.global_info['min_num_tracks'],y0,y1])
#        xlabel('3D error [scene diameter]')
#        ylabel('Number')

        show()

class AllMagErrors(object):
    def __init__(self, eval_data, global_info, extractor):
        self.extractor = extractor
        self.eval_data = eval_data
        self.info = global_info

    def calculate(self):
        eval_data = self.eval_data
        smin,smax = calculate_bounds_from_tracks(self.eval_data['tracks'])
        update_max_vector(self.info, 'smax', smax)
        update_min_vector(self.info, 'smin', smin)
        self.info['scale'] = 1.0/norm(self.info['smax']-self.info['smin'])

        # Smallest size
        min_num_tracks = len(self.eval_data['tracks'])
        update_min(self.info, 'min_num_tracks', min_num_tracks)

        self.all_mag_errors = all_mag_errors = []
        eval_data['all_residuals'] = array(eval_data['all_residuals'])
        norms = sqrt(sum(eval_data['all_residuals']**2, 0))
        norms.sort()
        # threshold at 4*median error
        error_thresh = norms[len(norms)/2]*4
        norms = norms[norms < error_thresh]
        self.norms = norms

    def plot(self):
        scale = self.info['scale']
        print 'scale=',scale
        color = self.eval_data['color']
#        norms = scale*self.norms
#        hist(scale*self.norms, sqrt(len(norms)),
#            normed=True,
#             label=self.eval_data['label'],
#             alpha=0.5,
#             lw=0,
#             fc=self.eval_data['color'],
#             )
        norms = stretch_to(scale*self.norms,1000)
        print self.eval_data['label'], norms[800]
        ttt = norms
        ttt.sort()
        x_new = arange(0.0, 1.0, 1.0/1000)
        plot(x_new, ttt,self.eval_data['color'], linewidth=3,
             label=self.eval_data['label']
            )

if __name__ == '__main__':
#if False:
    evals = []
    def add_eval(x, **extras):
        fn = '/data1/wrk/libmv_movies/blender_test1_%s/blender_test1.eval.json' % x
        new_eval_data = eval(file(fn).read())
        new_eval_data.update(extras)
        evals.append(new_eval_data)
#    add_eval('25px_buckets_scented_3view',  color='g', label='3-view, automatic keyframes')
    add_eval('25px_buckets_scented',        color='r', label='N-view, automatic keyframes')
    add_eval('25px_buckets_scented_fewkfs', color='m', label='N-view, manual keyframes')
    d = AllMagErrorsExtractor(evals)
    d.plot()
    show()

#if __name__ == '__main__':
if False: 
    evals = []
    def add_eval(x, **extras):
        new_eval_data = eval(file(x).read())
        new_eval_data.update(extras)
        evals.append(new_eval_data)
#    add_eval('/data1/wrk/libmv_movies/blender_test1_25px_no_buckets/eval.json',
#             color='g', label='27pix') # no buckets'
#    add_eval('/data1/wrk/libmv_movies/blender_test1_27px_buckets_replace/eval.json',
#             color='g', label='27pix, replacement') # buckets'
#    add_eval('/data1/wrk/libmv_movies/blender_test1_20px_buckets_replace/eval.json',
#             color='m', label='21pix, replacement') # buckets'
    add_eval('/data1/wrk/libmv_movies/blender_test1_25px/eval.json',
             color='b', label='27pix, no replacement') # buckets'
    add_eval('/data1/wrk/libmv_movies/blender_test1_20px_buckets/eval.json',
             color='r', label='21pix, no replacement') # buckets'

    add_eval('/data1/wrk/libmv_movies/blender_test1_27px_buckets_scented/eval.json',
             color='y', label='27pix, vanilla') # buckets'
    add_eval('/data1/wrk/libmv_movies/blender_test1_20px_buckets_scented/eval.json',
             color='c', label='21pix, vanilla') # buckets'
    d = DistanceFromFirstTrackExtractor(evals)
    d.plot()
    show()

"""
if False:
    evals = ['/data1/wrk/libmv_movies/blender_test1_25px_no_buckets/eval.json',
             '/data1/wrk/libmv_movies/blender_test1_25px/eval.json',
             '/data1/wrk/libmv_movies/blender_test1_20px_buckets/eval.json',
             '/data1/wrk/libmv_movies/blender_test1_20px_buckets_replace/eval.json']
    evals = [eval(file(x).read()) for x in evals]
    
    evals = []
    def add_eval(x, **extras):
        new_eval_data = eval(file(x).read())
        new_eval_data.update(extras)
        evals.append(new_eval_data)
#    add_eval('/data1/wrk/libmv_movies/blender_test1_25px_no_buckets/eval.json',
#             color='g', label='27pix') # no buckets'
    add_eval('/data1/wrk/libmv_movies/blender_test1_27px_buckets_replace/eval.json',
             color='g', label='27pix, replacement') # buckets'
    add_eval('/data1/wrk/libmv_movies/blender_test1_20px_buckets_replace/eval.json',
             color='m', label='21pix, replacement') # buckets'
    add_eval('/data1/wrk/libmv_movies/blender_test1_25px/eval.json',
             color='b', label='27pix, no replacement') # buckets'
    add_eval('/data1/wrk/libmv_movies/blender_test1_20px_buckets/eval.json',
             color='r', label='21pix, no replacement') # buckets'

    add_eval('/data1/wrk/libmv_movies/blender_test1_27px_buckets_scented/eval.json',
             color='y', label='27pix, vanilla') # buckets'
    add_eval('/data1/wrk/libmv_movies/blender_test1_20px_buckets_scented/eval.json',
             color='c', label='21pix, vanilla') # buckets'
    d = DistanceFromFirstTrackExtractor(evals)
    d.plot()
    show()
    """
