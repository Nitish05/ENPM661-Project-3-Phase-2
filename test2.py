import numpy as np

R = 0.33
RPM = int(input("Enter the RPM: "))


Lv = RPM*(2*np.pi*R)/60

 
print("The linear velocity is: ", Lv)

