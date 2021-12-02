from ConfigParser import SafeConfigParser

parser = SafeConfigParser()
parser.read('calib.ini')

pose = parser.get('CAMERA_PARAMS_LEFT2RIGHT_POSE', 'pose_quaternion')

pose = pose [1:-1]#remove brackets
li = list(pose.split(" "))#split up string
li_float = [float(a) for a in li]#convert to integer
print(li_float)