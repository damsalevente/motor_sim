import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('./plot.csv')

df.plot(x='t', y='wr')
plt.title("ud = 50V, uq = 50V")
plt.show()
