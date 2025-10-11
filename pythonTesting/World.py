from vpython import *
import random


class World:
    def __init__(self, config):
        self.width = 10.0
        self.height = 10.0
        self.num_gnomes = config["num_gnomes"]
        self.gnomes = []
        self.reset()

    def reset(self):

        self.gnomes = [(random.uniform(1.0, self.width - 1.0), random.uniform(1.0, self.height - 1.0)) 
                       for _ in range(self.num_gnomes)]