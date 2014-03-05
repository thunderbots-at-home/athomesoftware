import os, cPickle, logging
import numpy as np
from multiprocessing import Process
N = np
from patched_cifar10 import GridPatchCIFAR10
from pylearn2.train import Train
from pylearn2.models.autoencoder import DenoisingAutoencoder
from pylearn2.corruption import BinomialCorruptor
from pylearn2.training_algorithms.sgd import SGD
from pylearn2.costs.autoencoder import MeanSquaredReconstructionError
from pylearn2.termination_criteria import EpochCounter
from pylearn2.models.mlp import PretrainedLayer

class StackedDAE:


    def __init__(self):
        self.save_path ="./dae_random_patch_layer1_sub"
    # Parameters: None
    # What: Trains 5 denoising autoencoders with 512 hidden units each and then combines their weight
    # matrices to create a large 2560 layer that will be used as a transformer for the second layer.
    # the theory behind this is that the human visual system also creates an overcomplete basis    feature set in its first layer
    # Returns: Saves layer one in a .pkl file for use by another layer or transformer. 
    def create_layer_one(self):
        
        which_set = "train"
        one_hot = True
        start = 0
        # Creating 5 random patch layers based on 8,000 samples (Saturation point where the objective no longer improves.
        stop = 800
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
        cost = MeanSquaredReconstructionError()
        max_epochs = 10
        termination_criterion = EpochCounter(max_epochs=max_epochs)
        

        # SGD Learning algorithm
        algorithm = SGD(learning_rate=learning_rate, batch_size=batch_size, monitoring_batches=monitoring_batches, monitoring_dataset=dataset, cost=cost, termination_criterion=termination_criterion)


        processes = []
        for i in range(0,5):
            print "Training DAE Sub-Layer: ", i
            save_path = self.save_path+str(i)+".pkl"
            save_freq = 1
            train = Train(dataset=dataset,model=model,algorithm=algorithm, save_path=save_path, save_freq=save_freq)
            p = Process(target=train.main_loop, args=())
            p.start()
            processes.append(p)

        for process in processes:
            process.join()

    def combine_sublayers(self):
        print "Combining sub-layers"

        # Create a large 2560 unit DAE. The model is considered trained by the concatenation
        # of its 512 unit sub-layers. The 2560 hidden units weights will be initialized
        # by these. 
        nvis = 3072
        nhid = 2560
        irange = 0.05
        corruption = 0.2
        corruptor = BinomialCorruptor(corruption_level=corruption)
        activation_encoder = "tanh"
        activation_decoder = None

        # By default, the DAE initializes the weights at random
        # Since we're using our own already pre-trained weights
        # We will instead change those values.
        large_dae = DenoisingAutoencoder(nvis=nvis, nhid=nhid, corruptor=corruptor, irange=irange, act_enc=activation_encoder, act_dec=activation_decoder)

        # Do not need to change hidden or visible bias. 
        # They are static vars in theory.
        large_dae._params = [
            large_dae.visbias,
            large_dae.hidbias,
            # Here is where we change the weights.
            large_dae.weights
        ]

        numpy_array = np.zeros((3072, 2560))
        # Load sub-layer models and get their weights.
        for i in range (0,5):
            fo = open(self.save_path+str(i)+".pkl", 'rb')

            # 768 Vis, 512 hidden unit DAE.
            small_dae = cPickle.load(fo)
            fo.close()
            
            # TODO: Create numpy array of proper values to set the large_dae. 
            # Get the weights from the small_dae's
            # so that they can be appended together.                                      
            weights = small_dae.weights.get_value()
            large_dae_weights.append(weights)

        print "Successfully combined sub-layers"


        # Initialize a DAE with the large_dae_weights. 
dae = StackedDAE()
#dae.create_layer_one()

# Note: isn't called until layer one is created. 
dae.combine_sublayers()

