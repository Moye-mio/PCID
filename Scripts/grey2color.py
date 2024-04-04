import matplotlib.pyplot as plt
from PIL import Image
import numpy as np

ModelId = ["1", "2", "3", "4"]
FileName = ["ai.jpg", "dgi.png", "gt.png", "hole.png", "ours.png", "telea.png"]

def gray2color(gray_array, color_map):
    
    rows, cols = gray_array.shape
    color_array = np.zeros((rows, cols, 3), np.uint8)
 
    for i in range(0, rows):
        for j in range(0, cols):
            color_array[i, j] = color_map[gray_array[i, j]]
     
    return color_array

def test_gray2color():
    colormap = np.loadtxt("jet_int.txt", dtype = np.int32)

    for id in ModelId:
        for name in FileName:
            raw = Image.open("Images/" + id + "/" + name).convert("L")
            array = np.array(raw)
            color = gray2color(array, colormap)
            im = Image.fromarray(color)
            im.save("jet/" + id + "/" + name)
    return

test_gray2color()