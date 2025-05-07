import numpy

def checksum(u):
    cc = 0
    for c in u.strip().strip("$"): cc ^= ord(c)
    print(u+"*"+hex(cc)[2:].rjust(2, "0").upper())

# Flip on y-axis
flip_axis = 1

rotation_matrix = -numpy.eye(3)
rotation_matrix[flip_axis, flip_axis] = 1

Ax = -5.034 * 0.0254
Az = 19.676 * 0.0254

Bx = 57.803 * 0.0254
Bz = 7.25 * 0.0254

delta = 0.5 * 0.0254

checksum("$VNWRG,26,{}".format(",".join(str(x) for x in rotation_matrix.flatten())))
checksum("$VNWRG,57,{},0,{}".format(Ax, Az))
checksum("$VNWRG,93,{},00.000,{},{},{},{}".format(Bx-Ax, Bz-Az, delta, delta, delta))
# common    0
# time      0
# imu       1: 0600: Accel, AngularRate
# gnss      1: 0018: NumSats, GnssFix
# attitude  0
# ins       1: 0012: PosLla, VelNed
# gnss2     1: 0018: NumSats, GnssFix
checksum("$VNWRG,76,2,4,6C,0600,0018,0012,0018")
