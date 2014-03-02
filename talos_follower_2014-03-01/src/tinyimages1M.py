"""
.. todo::

    WRITEME
"""
import os, cPickle, logging
_logger = logging.getLogger(__name__)


import numpy as np
N = np
from pylearn2.datasets.dense_design_matrix import DenseDesignMatrix
from pylearn2.expr.preprocessing import global_contrast_normalize
from PIL import Image

def load_data(start, stop):
    # Loads the 1 million images into X and creates a DenseDesignMatrix
    # for use in a Denoising Autoencoder which is later used in a sDAE. 
    # Returns: dataset: DenseDesignMatrix(start, stop)
    #dataset_location = "~/catkin_ws/src/athomesoftware/datasets/tinyimages/"
    #dataset_location = "~/catkin_ws/src/athomesoftware/datasets/cifar10/"
    X = []
    y = []

    print("Loading images from " + dataset_location)
    for images in os.walk(dataset_location):
        if (images.endswith('.png')):
            im = Image.open(images)
            im.reshape(3, 32, 32)
            row = list(im.getData())
            X.append(row)
    
    print("Images loaded from " + dataset_location)
    X = np.asarray(X)
    y = np.asarray(y)
    y = y.reshape(y.shape[0], 1)
    X = X[start:stop, :]
    y = y[start:stop, :]

    print("Creating design matrix " + dataset_location)
    return DenseDesignMatrix(X=X, y=y)
        
