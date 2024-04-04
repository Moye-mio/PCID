import matplotlib.pyplot as plt
import numpy as np

import matplotlib
matplotlib.rc("font",family='FangSong')

date = ['Ridge', 'Reef', 'Peak', 'Mistral']
csf = [23.432, 12.141, 12.981, 6.886]
meshfix = [50.865 / 2, 44.728 / 2, 47.843 / 2, 29.735 / 2]
dgi = [694.99 / 2, 245.96 / 2, 597.585 / 2, 141.67 / 2]
nbfm = [98.138, 82.035, 102.682, 10.88]
ours = [221.68 / 2, 95.52 / 2, 228.17 / 2, 68.35 / 2]

x = np.arange(len(date))
width = 0.35

plt.bar(x - width/10*8, csf, width/5*2, label='CSF')
plt.bar(x - width/10*4, meshfix, width/5*2, label='MeshFix')
plt.bar(x, dgi, width/5*2, label='DGI')
plt.bar(x + width/10*4, nbfm, width/5*2, label='NBFM')
plt.bar(x + width/10*8, ours, width/5*2, label='本文方案')

plt.ylabel('时间开销 (s)',fontweight='bold')
plt.xlabel('场景',fontweight='bold')
# plt.title('Test Result Trend')
plt.xticks(x,date)
plt.legend()
plt.show()
