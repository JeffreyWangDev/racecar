import tensorflow as tf

from tensorflow.keras import datasets, layers, models
import matplotlib.pyplot as plt
import pandas as pd

def read_csv(name):
    df = pd.read_csv(filepath, names=['sentence', 'label'], sep='\t')