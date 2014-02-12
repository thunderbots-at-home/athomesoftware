#If you add any libraries or supporting libraries for compilation, please add the dependency name here of the commands you used to install it and its name. This will be useful for all others. Thank you.

# Installing Python 3.
sudo apt-get install python3 

# Installing a dependency for theano
sudo apt-get install python-scipy

# Another dependency for theano
sudo apt-get install python-numpy

# For easy installing of python libraries
sudo apt-get install python-pip

# Installing theano, required for pylearn2
pip install git+http://github.com/Theano/Theano.git

# py-yaml is required for pylearn2
svn checkout http://svn.pyyaml.org/pyyaml/trunk pyyaml-trunk

# Installation of pylearn2
git clone git://github.com/lisa-lab/pylearn2.git
# required for finding datsets in your code
export PYLEARN2_DATA_PATH=/data/lisa/data

# Installing PIL, dependency of pylearn2 for image functionality
sudo git clone https://github.com/python-imaging/Pillow.git




