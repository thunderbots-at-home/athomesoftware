from pylearn2.config import yaml_parse

layer1_yaml = open('tracker_dae_l1.yaml', 'r').read()
train = yaml_parse.load(layer1_yaml)
train.main_loop()
