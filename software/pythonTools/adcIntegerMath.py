import matplotlib.pyplot as plt
import numpy as np

ADCres = 4096
adcVal = 1000

currentBias = 20
currentScale = 40

ampsReal = ((adcVal / ADCres) * currentScale)-currentBias

print(ampsReal)
