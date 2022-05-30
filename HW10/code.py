from ulab import numpy as np

list1 = np.empty([1024])
for i in range(len(list1)):
    a = np.sin(3*i)
    b = np.sin(40*i)
    c = np.sin(100*i)
    list1[i]= a+b+c
    #sp = np.fft.fft(a+b+c)
    print("("+str(list1[i])+",)")

#newarr = np.fft.fft(list1)
#print("("+str(newarr)+",)")
