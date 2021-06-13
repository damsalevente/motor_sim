import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('./plot.csv')
df['p'] = df['id'] * df['ud']

df.plot(x='t', y=['wr', 'te'])
plt.title("ud = 50V, uq = 50V")
plt.show()

df.plot(x='wr', y='te')
plt.show()


df.plot(x='t', y=['id', 'iq'])
plt.show()
