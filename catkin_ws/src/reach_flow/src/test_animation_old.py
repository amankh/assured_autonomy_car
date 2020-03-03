import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle


fig = plt.figure()
#creating a subplot 
ax1 = fig.add_subplot(1,1,1)

def isfloat(value):
    try:
        float(value)
        return True
    except ValueError:
        return False

def animate(i):
    data = open('flow.txt','r').read()
    lines = data.split('\n')
    #print lines
    xc = []
    yc = []
    width = []
    height = []
    #print lines
    i = 0
    for line in lines:
        a = line.strip().split(' ') # Delimiter is comma
        #print 'a0', a[0], isfloat(a[0])
        #break
        #print a[1]
        if isfloat(a[0])== True and isfloat(a[1])== True:
            xc.append(float(a[0])*100)
            yc.append(float(a[1])*100)
            width.append(float(a[2])*100)
            height.append(float(a[3])*100)
            #print xs[-1], ys[-1]
        #print xc
        if i <= len(xc)-1:
            plt.gca().add_patch(plt.Rectangle((xc[i], yc[i]), width[i], height[i], fill=False, linewidth=1.5))
        #ax1.clear()
        #ax1.plot(xs, ys)
        plt.xlabel('X_position')
        plt.ylabel('Y_position')
        plt.xlim(0, 10)
        plt.ylim(-10, 10)  
        plt.title('Reachable states')
        i += 1 
    #print xc
    
ani = animation.FuncAnimation(fig, animate, interval=500)
plt.show()
