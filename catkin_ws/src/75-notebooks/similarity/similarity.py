#!/usr/bin/env python2
from duckietown_utils import bgr_from_rgb, rgb_from_bgr
from duckietown_utils import write_image_as_jpg
from duckietown_utils import image_cv_from_jpg_fn
from duckietown_utils import locate_files
from duckietown_utils import safe_pickle_dump
import os

from quickapp.quick_app import QuickApp
from reprep import Report
from reprep.graphics.filter_scale import scale

import numpy as np
from duckietown_utils.image_rescaling import d8_image_zoom_linear

class SimilarityMatrix(QuickApp):

    def define_options(self, params):
        params.add_string('dirname', help="Input directory") 
        
    def define_jobs_context(self, context):
        
        dirname = self.options.dirname
        filenames = locate_files(dirname, '*.jpg')
        if len(filenames) == 0:
            msg = 'Could not find any file'
            raise Exception(msg)
        options = {
            'L1': dict(phi=make_smaller, distance=L1),
            'L2': dict(phi=make_smaller, distance=L2),
        }
        
        filenames = [ f for f in filenames if '-0' in f]
        filenames = sorted(filenames)
        print "\n".join(filenames)
        
        for id_option, params in options.items():
            images = [image_cv_from_jpg_fn(f) for f in filenames]
            c= context.child(id_option)
            out = os.path.join(dirname, 'similarity', id_option, 'similarity')
            A = c.comp(get_similarity_matrix, images, out=out, **params)
            c.comp(write_similarity_matrix, A, out+'_final', more=True, images=images)   

def make_smaller(x):
    x = d8_image_zoom_linear(x, 1/16.0)
    return  x.astype('float32')

def asfloat(x):
    return x.astype('float32')



def L1(a, b):
    d = np.mean(np.fabs(a-b))
    return d

def L2(a, b):
    diff = np.fabs(a-b)
    d = np.sqrt(np.sum(diff*diff))
    H, W, S = a.shape
    d = d / (H*W*S)
    return d


def get_similarity_matrix(images, phi, distance, out):
    n = len(images)    
    descriptors = [phi(image) for image in images]
    A = np.zeros((n,n))
    for i in range(n):
        for j in range(n):
            if i <= j:
                d = distance(descriptors[i], descriptors[j])
                A[i,j] = d
                A[j,i] = d
        
        if i % 3 == 0:
            write_similarity_matrix(A, out)
            
    for i in range(n):
        for j in range(n):
            if j < i:
                A[i,j] = A[j,i]
    return A

def write_similarity_matrix(A, out, more=False, images=None):
    safe_pickle_dump(A, out+'.pickle')
#     rgb = rgb_zoom(scale(A), 8)
    rgb = scale(A)
    rgb = d8_image_zoom_linear(rgb, 8)
    write_image_as_jpg(rgb, out + '.jpg')
    
    if more:
        r = Report()
        n = A.shape[0]
        for i in range(n):
            f = r.figure()
            
            ignore = 10
            Ai = A[i, :].copy()
            for i2 in range(i-ignore, i+ignore):
                if 0 <= i2 < n:
                    Ai[i2] = np.inf
            jbest = np.argmin(Ai)
            
            with f.plot('i-%d' % i) as pylab:
                pylab.plot(A[i, :], '-*')
            
            if images:    
                f.data_rgb(str(i), bgr_from_rgb(images[i]))
                f.data_rgb(str(jbest), bgr_from_rgb(images[jbest]))
            
        r.to_html(out+'.html')
    
if __name__ == '__main__':
    main = SimilarityMatrix.get_sys_main()
    main()