import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

df = pd.read_csv('data/logFOC.csv')  # , index_col=0)
dfbug = pd.DataFrame()
# df['sinTheta'] = np.sin(-df.theta)
# df['cosTheta'] = np.cos(-df.theta)
print(df.describe())
print(df.head())
df['cCalc'] = -df.a - df.b
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)


dfbug['theta'] = (df.theta / np.power(2,14)) * 2*np.pi

dfbug['a'] = 512 * np.sin(dfbug.theta - (np.pi/2))

# Power invariant clarke
dfbug['X'] = df.a #(2 * df.a - df.b - df.c) * (1 / np.sqrt(6))
dfbug['Y'] = (df.b - df.c) * (1 / np.sqrt(3))

# Power Variant clarke
dfbug['Xvar'] = (2 * df.a - df.b - df.c) * (1 / 3)
dfbug['Yvar'] = (df.b - df.c) * (1 / np.sqrt(3))


dfbug['D'] = dfbug.X * np.cos(dfbug.theta) + dfbug.Y * np.sin(dfbug.theta)
dfbug['Q'] = dfbug.Y * np.cos(dfbug.theta) - dfbug.X * np.sin(dfbug.theta)

#ax1.plot(df.index, df.theta)
ax1.plot(df.index, df.a, color='red')
ax1.plot(dfbug.index, dfbug.a, color='red')

ax1.plot(df.index, df.b, color='green')
ax1.plot(df.index, df.c, color='blue')
#ax1.plot(df.index, df.cCalc, color='grey')
#ax1.plot(df.index, df.a + df.b + df.c)

ax1.set(xlabel='sample', ylabel='value', title='phases')
ax1.grid()
# ax1.legend()
# plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=0.1)


#ax2.plot(df.index, df.alpha, lw=0.5, label='alpha')
#ax2.plot(df.index, df.beta, lw=0.5,label='beta')
ax2.plot(dfbug.index, dfbug.X, label='X')
ax2.plot(dfbug.index, dfbug.Y, label='Y')
#ax2.plot(df.index, df.Id, label='Id')
#ax2.plot(df.index, df.Iq, label='Iq')
#
# ax2.set(xlabel='sample', ylabel='value', title='Clarke')
ax2.grid()
ax2.legend()
#
#ax3.plot(df.index, df.Id, label='Id')
#ax3.plot(df.index, df.Iq, label='Iq')
ax3.plot(dfbug.index, dfbug.D, label='Id')
ax3.plot(dfbug.index, dfbug.Q, label='Iq')
#
# ax3.set(xlabel='sample', ylabel='value', title='Park')
ax3.grid()
ax3.legend()

plt.show()
