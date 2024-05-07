# BCIT-HEXAPOD ROBOTICS CLUB 
# Author(s): Hassan Islam 
# Date : 2024-04-24
# DESC : This following implements Newton's Method to numerically solve for the inverse kinematics of a 3-DOF 
#        robotic arm that otherwise has no analytical solution. This is done in order to find the joint
#        angles required to position the end effector (wrist) of the robotic arm at a given Cartesian position 
#
# NAMING CONVENTIONS NOTE:
#               * Y-coordinate refers to height, X-coordinate is forward discplacement of arm, Z-coordinate is lateral displacement
#               * theta angle is between z and x axis (Transverse plane), gamma is angle between first and second link, phi is angle between second and third link, 
#
# FUTURE REFACTOR NOTE: Coordinate2JointAngle uses Radians for its initial position, but outputs degrees. Changes both to degrees --- FIXED


''' Example implementation

    x = np.array([np.pi/4,np.pi/4,np.pi/4], dtype=float) # Specify initial positon of arm in radians. Usually the startup position
    l = np.array([1, 1, 1])                              # Specify link lengths 
    p = np.array([0,0,3])                                # Target position in cartesion coordinates

    [finalPosition, iterations] = Coordinate2JointAngle(x, l, p, True) # True --> Newtons Method iterations are returned 

    print("\n{ gamma, theta, phi } =",format(finalPosition), "\n")

    if iterations is not None:
        for position in iterations:
            print("\n",position) 

'''
import autograd.numpy as np
from autograd import jacobian
from numpy.linalg import inv

# for neatness, the following functions have been given aliases 
c = np.cos
s = np.sin

# Forward kinematics of arm (precalculated by hand for our specific 3-DOF robotic arm) 
#        theta : angle of revolute (shoulder)   
#        gamma : angle of revolute (elbow)
#        phi : angle of end effector (wrist)
#        l[0] : length of end effector
#        l[1] : length of forearm (elbow to wrist joint)
#        l[2] : length of arm (shoulder to elbow joint)
def func(x, l, p): 
    theta, gamma, phi = x 
    return np.array([
        l[0]*(c(gamma)*c(theta)*c(phi) - s(gamma)*c(theta)*s(phi)) + l[1]*c(gamma)*c(theta) + l[2]*c(theta) - p[0],
        l[0]*(c(gamma)*s(phi) + s(gamma)*c(phi)) + l[1]*s(gamma) - p[1],
        -l[0]*(s(gamma)*s(theta)*s(phi) - c(gamma)*s(theta)*c(phi)) + l[1]*c(gamma)*s(theta) + l[2]*s(theta) - p[2]
    ])

def InverseKinematicsNewtonsMethod(func, J, initialAngluarposition, linkLengths, targetPosition, maxIterations=100, tol=1e-4):
    position = initialAngluarposition

    iterations = [position]

    # iterate using x[n+1] = x[n] + J^-1*f[x]
    for i in range(maxIterations):
        func_Jacobian = J(position, linkLengths, targetPosition)  # Pass the required arguments to func
        
        # Check if Jacobian determinant is close to zero
        if np.linalg.norm(np.linalg.det(func_Jacobian)) < tol:
            print("\nJacobian is singular. Exiting.")
            return None, None
        
        inv_jacobian = inv(func_Jacobian)
        positionIteration = position - np.matmul(inv_jacobian, func(position, linkLengths, targetPosition))
        
        if np.linalg.norm(func(positionIteration, linkLengths, targetPosition)) < tol:
            return positionIteration, iterations
        
        position = positionIteration

        iterations.append(position)
    
    # iteration limit to avoid infinite loops. Should not occur. Highley likely solution is inaccurate if it does. 
    print("\nWarning: Max iterations reached. Solution might not be accurate.")
    return position, iterations

# function returns Angle position of target position 
# args
#    initialAngluarposition : specificy the current position of the robotic arm in radians
#    linkLengths: specificy length of arm links
#    targetCartesionPosition: desired end position of robotic arm
# returns
#   position : angluar position (degrees) of target positon [ gamma, theta, phi ]
#   iterations: outputs all iterations This can allow for the robotic arm to move smoothly to its positon
def Coordinate2JointAngle(initialAngluarposition, linkLengths, targetCartesionPosition, returnIterations):

    x = initialAngluarposition * np.pi/180 # argument is in degrees, must be converted to radians. 
    l = linkLengths
    p = targetCartesionPosition

    position, iterations = InverseKinematicsNewtonsMethod(func, jacobian(func), x, l, p)

    # occurs if jacobian is singular
    if not position.any() or not iterations:
        return None, None
    
    # test if f(x) = 0
    f = func(position, l, p)

    tolerance = 1e-5

    if all(abs(value) < tolerance for value in f):
        print("") # \print("\nf(x) =", f, "Solution converges")
    else:
        print("\nf(x) =", f, "Solution does not converge to 0. Target Position may be impossible")

    # conversion to degrees
    position = position * 180/np.pi

    # normalizing angle 
    for i in range(len(position)):
        angle = position[i] % 360

    # Force it to be the positive remainder, so that 0 <= angle < 360
        angle = (angle + 360) % 360

    # Force into the minimum absolute value, so that -180 < angle <= 180
        if angle > 180:
            angle -= 360

        position[i] = angle

    if returnIterations:
        iterations_normalized = []

        for position in iterations:
            position = position * 180/np.pi

            for i in range(len(position)):
                angle = position[i] % 360

                # Force it to be the positive remainder, so that 0 <= angle < 360
                angle = (angle + 360) % 360

                # Force into the minimum absolute value, so that -180 < angle <= 180
                if angle > 180:
                    angle -= 360

                position[i] = angle

            iterations_normalized.append(position)

        # last element of iterations_normalized is the same as position, so it is omitted. 
        return position, iterations_normalized[:len(iterations_normalized) - 1]
    else:
        return position, None


