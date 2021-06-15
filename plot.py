import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('./plot.csv')
df['p'] = df['id'] * df['ud']

df.plot(y=['wr', 'te'])
plt.title("")
plt.show()

df.plot(x='wr', y='te')
plt.show()


df.plot(y=['id', 'iq'])
plt.show()
