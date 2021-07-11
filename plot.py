import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('./plot.csv')
df['p'] = df['id'] * df['ud']

df.plot(x = 't', y = ['wr', 'te'])
plt.xlabel('time[0.01s]')
plt.show()
