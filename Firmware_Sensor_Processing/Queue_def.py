import numpy as np

class FixedLengthQueue:
    def __init__(self, max_length, element_length):
        self.max_length = max_length
        self.element_length = element_length
        self.queue = np.zeros((max_length, element_length))
        self.current_index = 0

    def push(self, new_element):
        self.queue = np.roll(self.queue, -1, axis=0)  # Left shift the entire array
        self.queue[-1] = new_element

    def get_array(self):
        return self.queue

