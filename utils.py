"""
Standard utilies
You can use this function if you have an input value and want to make sure it's in a specified range. 
For example, if you have a joystick value that's 1.1 and you called the clamp function with a min of 0 and a max of 1
it will change the joystick value from 1.1 to 1.
"""

def clamp(value, max, min): # this function takes the input value and your specified range 
    return max if value > max else min if value < min else value 
    # if the value is above the specified max it will just change it to the max value 
    # and visa versa, if the value is below the min it will just change it to the minimum value 
