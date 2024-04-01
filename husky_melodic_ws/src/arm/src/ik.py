import unittest
from math import sqrt, atan2, acos, asin

# Import the function you want to test
def ik_geo(Xc, Yc, Zc):
    r = sqrt(Xc**2 + Yc**2)
    d = Xc/(r)
    # offset = asin(0.03/(r-0.07))
    theta1_1 = atan2(sqrt(1-d**2), d)
    theta1_2 = atan2(-sqrt(1-d**2), d)
    if Yc >= 0 :
        if theta1_1 > 0:
            theta1 = theta1_1
        else: theta1 = theta1_2
    else: 
        if theta1_1 < 0:
            theta1 = theta1_1
        else: theta1 = theta1_2

        
    alpha = atan2((r-0.07),(Zc-0.5347))
    a = 0.3701
    b = sqrt((Zc - 0.5347)**2 + (r-0.07)**2)
    c = sqrt((0.45)**2 + (0.0301)**2)
    beta = acos((a**2 + b**2 - c**2)/(2*a*b))
    print("alpha:", alpha)
    print("beta", beta)
    theta2 = alpha - beta
    omega = acos((a**2 + c**2 - b**2)/(2*a*c))
    delta = atan2(0.45,0.0301)
    theta3 = 3.1415926535 - omega - delta

    print(f"Test case: Xc={Xc}, Yc={Yc}, Zc={Zc}")
    print(f"Theta1: {theta1/3.1415926 * 180}")
    print(f"Theta2: {theta2/3.1415926 * 180}")
    print(f"Theta3: {theta3/3.1415926 * 180}")


    return theta1, theta2, theta3

