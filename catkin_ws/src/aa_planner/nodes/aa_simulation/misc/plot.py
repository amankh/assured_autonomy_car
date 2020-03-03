#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import sys
y = np.array([float(x) for x in sys.argv[1:]])
x = np.array(range(len(y)))
x2 = x[len(y) // 2:]
y2 = y[len(y) // 2:]
f1=np.polynomial.polynomial.Polynomial.fit(x,y,1)
f2=np.polynomial.polynomial.Polynomial.fit(x2,y2,1)
xnew = np.linspace(0, len(x)*2, num=len(x) * 4 + 1, endpoint=True)
xnew2= np.linspace(len(x) // 2, len(x)*2, num=len(x) * 3 + 1, endpoint=True)
plt.plot(x, y, 'o', xnew, f1(xnew), '-', xnew2, f2(xnew2), '--')
plt.show()

