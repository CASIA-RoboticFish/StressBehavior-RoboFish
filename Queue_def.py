import numpy as np

class FixedLengthQueue:
    def __init__(self, max_length, element_length):
        self.max_length = max_length
        self.element_length = element_length
        self.queue = np.zeros((max_length, element_length))
        self.current_index = 0

    def push(self, new_element):
        self.queue = np.roll(self.queue, -1, axis=0)  # 左移整个数组
        self.queue[-1] = new_element

    def get_array(self):
        return self.queue

# # 创建一个长度为50的先入先出数组，每个位置存储六个元素
# array_length = 50
# element_length = 6
# fifo_array = FixedLengthQueue(array_length, element_length)
#
# # 模拟不断更新数组
# for i in range(100):
#     new_element = np.array([i, i+1, i+2, i+3, i+4, i+5])
#     fifo_array.push(new_element)
#
# # 获取最终的先入先出数组
# final_array = fifo_array.get_array()
#
# # 打印结果
# for i, element in enumerate(final_array):
#     print(f"Position {i}: {element}")
