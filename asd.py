import numpy as np
def convolution(array: np.ndarray, kernel: list) -> np.ndarray:
    kernel = kernel[::-1]
    end_array = np.zeros(array.size)
    array = np.pad(array, (len(kernel)//2, len(kernel)//2), 'constant')
    for i in range(array.size-((len(kernel)//2) *2)):
        end_array[i] = np.sum(array[i:i+len(kernel)] * kernel)
    return end_array
a = np.array([2, 4, 1, 5, 2, 7, 6, 8, 1, 0, 2, 2, 1, 5, 2, 4, 8, 6, 1, 10])
b=  [1,0, 2, 0, 4, 0, 8,0]

d = np.convolve(a, b)
print(d)
print(len(d))
print(sum(d))
print(27*4/24)