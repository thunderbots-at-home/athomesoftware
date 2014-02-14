import os, cPickle, logging
_logger = logging.getLogger(__name__)

import numpy as np
N = np
from patched_cifar10 import GridPatchCIFAR10
from pylearn2.train import Train
from pylearn2.models.autoencoder import DenoisingAutoencoder
from pylearn2.corruption import BinomialCorruptor
from pylearn2.training_algorithms.sgd import SGD
from pylearn2.costs.autoencoder import MeanSquaredReconstruction
from pylearn2.termination_criteria import EpochCounter

class StackedDAE:

    def __init__(self):
        
    # Parameters: None
    # What: Trains 5 denoising autoencoders with 512 hidden units each and then combines their weight
    # matrices to create a large 2560 layer that will be used as a transformer for the second layer.
    # the theory behind this is that the human visual system also creates an overcomplete basis    feature set in its first layer
    # Returns: Saves layer one in a .pkl file for use by another layer or transformer. 
    def create_layer_one():
        
        which_set = "train"
        one_hot = True
        start = 0
        # Creating 5 random patch layers based on 8,000 samples (Saturation point where the objective no longer improves.
        stop = 8000
        # GridPatchCIFAR10 Randomly selects 5 16x16 patches from each image, and we do this 5 times. This helps increase training time and captures more information. Similar to how the neurons in the eye are attached to a specific region in the image.
        dataset = GridPatchCIFAR10(which_set=which_set, one_hot=one_hot, start=start, stop=stop)

        # Denoising autoencoder model hyper-parameters
        nvis = 768
        nhid = 512 
        irange = 0.05
        corruption_lvl = 0.2
        corruptor = BinomialCorruptor(corruption_level=corruption_lvl)
        activation_encoder = "tanh"
        # Linear activation
        activation_decoder = None 

        # Creating the denoising autoencoder
        model = DenoisingAutoencoder(nvis=nvis, nhid=nhid, irange=irange, corruptor=corruptor, act_enc=activation_encoder, act_dec=activation_decoder)

        # Parameters for SGD learning algorithm instantiated below
        learning_rate = 0.001
        batch_size = 100
        monitoring_batches = 5
        monitoring_dataset = dataset
        cost = MeanSquaredReconstruction()
        max_epochs = 10
        termination_criteria = EpochCounter(max_epochs=max_epochs)
        

        # SGD Learning algorithm
        algorithm = SGD(learning_rate=learning_rate, batch_size=batch_size, monitoring_batches=monitoring_batches, monitoring_dataset=monitoring_dataset, cost=cost, max_epochs=max_epochs, termination_criteria=termination_criteria)

        for i in range(0,5)
            print("Training DAE Sub-Layer: " + i)
            save_path = "./dae_random_patch_layer1_sub" + itr + ".pkl"
            save_freq = 1
            train = Train(dataset=dataset,model=model,algorithm=algorithm, save_path=save_path, save_freq=save_freq)
    
            train.main_loop()
