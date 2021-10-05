from geometry_msgs.msg import Quaternion

def calcOrientation(theta):
    v = (0,0,1)
    orientation = Quaternion()
    orientation.x = v[0]*math.sin(theta/2)
    orientation.y = v[1]*math.sin(theta/2)
    orientation.z = v[2]*math.sin(theta/2)
    orientation.w = math.cos(theta/2)

    return orientation
