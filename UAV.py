import numpy as np
import scipy
import math
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
R=107 #修改无人机编队圆周半径值
loc=[[0,0],[100,0],[98,40.10],[112,80.21],[105,119.75],[98,159.86],[112,199.96],[105,240.07],[98,280.17],[112,320.28]]#记录每架无人机的实际位置
nloc=[[0,0],[R,0],[R,40],[R,80],[R,120],[R,160],[R,200],[R,240],[R,280],[R,320]]#记录每架无人机的理想位置
err=[0]*10 #取实际点与理想点的距离平方为误差
e=[1]*10
res=[[0 for col in range(2)] for row in range(10)]

A1=[i[0] for i in loc]
B1=[np.radians(i[1]) for i in loc]

#根据半径选择fsolve函数的初始值
if R==99 or R==100 or R==101 or R==102:
    x1=100
    x2=100
    x3=100
elif R==103 or R==104 or R==105 or R==106:
    x1=70
    x2=100
    x3=120
elif R==107 or R==108:
    x1=10
    x2=120
    x3=120
elif R==98 or R==109 or R==110 or R==111 or R==112:
    x1=10
    x2=50
    x3=50
#定义f(X)函数，计算无人机所去位置的方向信息
def f(X):
    caerfa1=X[0]
    caerfa2=X[1]
    caerfa3=X[2]
    a=R
    return [a**2+c**2-2*a*c*caerfa3-R**2,
            a**2+b**2-2*a*b*caerfa1-R**2,
            R**2+R**2-2*R*R*np.cos(math.radians(abs(theta2-theta1)))-b**2-c**2+2*b*c*caerfa2]
#定义g(X)函数，定位无人机接收信号后调整所去的位置
def g(X):
    a=X[0]
    b=X[1]
    c=X[2]
    ctheta=X[3]
    r1=loc[m][0]
    r2=loc[n][0]
    return [a**2+c**2-2*a*c*angle[2]-r2**2,
            a**2+b**2-2*a*b*angle[0]-r1**2,
            r1**2+r2**2-2*r1*r2*np.cos(math.radians(abs(theta2-theta1)))-b**2-c**2+2*b*c*angle[1],
            a**2+loc[m][0]**2-2*a*loc[m][0]*ctheta-b**2]

for i in range(1,10):
    err[i]=loc[i][0]**2+nloc[i][0]**2-2*loc[i][0]*nloc[i][0]*np.cos(math.radians(abs(loc[i][1]-nloc[i][1])))
#通过迭代，对无人机调整队形
for j in range(1):
    sorted_id=sorted(range(len(err)), key=lambda k: err[k], reverse=False)
    print(sorted_id)
    # sorted_id[1],sorted_id[2],0三架无人机发射信号调整其余无人机位置
    for i in range(1,10):
        if i==sorted_id[1] or i==sorted_id[2]:
            continue
        if sorted_id[1]<sorted_id[2]:
            m=sorted_id[1]
            n=sorted_id[2]
        else:
            m=sorted_id[2]
            n=sorted_id[1]
        b=math.sqrt(2*R**2-2*R*R*np.cos(math.radians(abs(nloc[i][1]-nloc[m][1]))))
        c=math.sqrt(2*R**2-2*R*R*np.cos(math.radians(abs(nloc[m][1]-nloc[n][1]))))
        theta1=nloc[m][1]
        theta2=nloc[n][1]
        angle= fsolve(f,[0,1,0],maxfev=50000)
        #print(angle)
        result=fsolve(g, [x1,x2,x3,0],maxfev=50000)
        #print(math.degrees(np.arccos(result[3])))
        #print(loc[1][1]-math.degrees(np.arccos(result[3])))
        #print(result)
        res[i][0]=result[0]
        if abs((nloc[i][1]-(loc[m][1]-math.degrees(np.arccos(result[3])))-360))%360<10 or abs((nloc[i][1]-(loc[m][1]-math.degrees(np.arccos(result[3])))+360))%360<10:
            res[i][1]=(loc[m][1]-math.degrees(np.arccos(result[3]))+720)%360
        else:
            res[i][1]=(loc[m][1]+math.degrees(np.arccos(result[3]))+720)%360
    for i in range(1,10):
        if i==sorted_id[1] or i==sorted_id[2]:
            continue
        loc[i][0]=res[i][0]
        loc[i][1]=res[i][1]#更新位置
    print(loc)
    for i in range(1,10):
        err[i]=loc[i][0]**2+nloc[i][0]**2-2*loc[i][0]*nloc[i][0]*np.cos(math.radians(abs(loc[i][1]-nloc[i][1])))#更新误差
    e[j]=np.sum(err)
    print(e[j])
print((e[0]-e[1])/e[0])
#绘图，蓝色点为初始位置，红色点为调整后的位置
A2=[i[0] for i in loc]
B2=[np.radians(i[1]) for i in loc]
N=0#空的极坐标图
plt.figure(figsize=(10,5), dpi=80)#创建一个画布，也可以不使用
theta=np.linspace(0.0,2*np.pi ,N, endpoint=False)
radii=10*np. random. rand(N)
width=np.pi/4* np.random.rand(N)
ax=plt. subplot(111, projection='polar')
bars=ax.bar(theta, radii , width=width, bottom=0.0)
for r,bar in zip(radii,bars):
    bar.set_facecolor(plt.cm.viridis(r/10.))
    bar.set_alpha(0.5)
ax.set_yticks([20,40,60,80,100,120])
ax.plot(B1,A1,'bs')
ax.plot(B2,A2,'r^')
plt.title(f'R={R}')
plt. show()
