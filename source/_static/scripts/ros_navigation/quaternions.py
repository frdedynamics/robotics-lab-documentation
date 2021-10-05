from geometry_msgs.msg import Quaternion

def calcOrientation(theta):
    orientation = Quaternion()
    orientation.x = v[0]*math.sin(theta/2)
    orientation.y = v[1]*math.sin(theta/2)
    orientation.z = v[2]*math.sin(theta/2)
    orientation.w = math.cos(theta/2)

    return orientation
