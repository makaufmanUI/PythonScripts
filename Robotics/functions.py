#####
#
#   Functions from the Spring 2022 semester, Robotics and Automation
#
#####


import numpy as np
import pandas as pd
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.linalg import expm, sinm, cosm
np.set_printoptions(precision=3, suppress=True)
pd.options.display.float_format = '{:,.3f}'.format


I3, I4 = np.identity(3), np.identity(4)
sin, cos = np.sin, np.cos
deg, rad = np.degrees, np.radians
matrix, matmul = np.matrix, np.matmul
pi, exp, sqrt = np.pi, np.exp, np.sqrt
asin, acos, atan = np.arcsin, np.arccos, np.arctan2





# cotangent function (cos/sin)
def cot(theta):
	val = np.cos(theta) / np.sin(theta)
	return val



def JacobianSpace(S = [], theta = []):
	"""Generates the Space Jacobian ( 6 x 1 ) for a specific joint (need to combine Jacobians for all 𝑛 joints to get full 6 x 𝑛 Jacobian matrix -- example in 𝑛𝑜𝑡𝑒𝑠.𝑝𝑦)

	Args:
	- S (`list`, `np.array`): List of joint Screws ( 6 x 1 ) up to and including the joint # in question; i.e., the length of the S list MUST be EQUAL to the joint # in quesiton.
		The first Screw in S MUST BE the Screw at joint #1. (S₁, S₂, ..., Sₙ), where n = joint # in question.
	- theta (`list`, `radians`): List of joint angles for each joint up to and NOT INCLUDING the joint in question;
		i.e., the length of the theta list MUST be ONE LESS than the length of the S list. (𝜃₁, 𝜃₂, ..., 𝜃ₙ₋₁), where n = joint # in question.
	
	Returns:
		𝐽sᵢ (`np.array`): Space Jacobian ( 6 x 1 ) for the joint in question (𝑖)
	"""
	for i in range(len(S)): S[i].shape = (6,1)
	Js1, S1 = S[0], S[0]
	
	exponentials = []
	for i in range(len(S) - 1):
		exponentials.append( RBMexponential(S[i], theta[i]) )
	
	PoE = I4
	for i in range(len(exponentials)):
		PoE = PoE * exponentials[i]
	
	AdjPoE = Adjoint2(PoE)
	Js_i = AdjPoE * S[len(S) - 1]

	return Js_i



def JacobianBody(Jspace, R, p):
	Tbs = Transform(R,p)
	Jbody = Adjoint2(Tbs) * Jspace
	return Jbody



def JacobianBody2(Jspace, Tbs):
	Jbody = Adjoint2(Tbs) * Jspace
	return Jbody




# Combines the RBMexponential function to compute a product of exponentials in one go
def RBMPoE(frame, M, S = [], theta = []):
	"""Performs the RBM Product of Exponentials to express the final configuration
	of a robot given the frame you want this configuration expressed in (𝐟𝐫𝐚𝐦𝐞), a Home Position (𝐌),
	a list of Screws for each joint where 𝜃 ≠ 0 ( [ 𝑆₁ , 𝑆₂ , ... ] ), and a list of angles corresponding to those joints ( [ 𝜽₁ , 𝜽₂ , ... ] )


	Args:
	- 𝐟𝐫𝐚𝐦𝐞 (`string`): The frame the final configuration is to be expressed in (either '𝑠𝑝𝑎𝑐𝑒' or '𝑏𝑜𝑑𝑦'). Determines whether the Home Position (𝐌) is 𝑟𝑖𝑔𝘩𝑡 or 𝑙𝑒𝑓𝑡 multiplied, respectively.
	- 𝐌 (`np.matrix`): Matrix ( 4 x 4 ) describing the Home Position of the robot (the orientation of the end-effector wrt the base). Combination of a rotation matrix 𝑅 and a location vector 𝑝.
	- 𝐒 (`np.array` `list`): Screw ( 1 x 6 ) vectors for each joint where 𝜃 ≠ 0
	- 𝐭𝐡𝐞𝐭𝐚 (`int`/`float` `list`): Rotation angles of the joints where 𝜃 ≠ 0, in 𝑟𝑎𝑑𝑖𝑎𝑛𝑠

	Returns:
		T (`np.matrix`): Final configuration (transformation matrix) ( 4 x 4 )
	"""
	exponentials = []			# empty list to store each rigid-body exponential result
	for i in range(len(S)):
		exponentials.append( RBMexponential(S[i], theta[i]) )	# append exponentials to list
	
	product = I4				# initialize the product of exponentials as identity matrix
	for i in range(len(exponentials)):
		product = product * exponentials[i]					# compute the product of exponentials
	
	if frame == 'space':
		T = product * M			# right-multiply by M if expressed in space frame
		T = np.where( matrix.round(T,3) == 0, 0, T )	# gets rid of negative signs in front of zeros
		return T
	
	elif frame == 'body':
		T = M * product			# left-multiply by M if expressed in body frame
		T = np.where( matrix.round(T,3) == 0, 0, T )	# gets rid of negative signs in front of zeros
		return T





def RBMexponential(S, theta):
    """Returns the rigid-body exponential (4x4) given a Screw and theta

	Args:
		𝐒 (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Screw ( 1 x 6 )
		𝐭𝐡𝐞𝐭𝐚 (𝑖𝑛𝑡/𝑓𝑙𝑜𝑎𝑡): Distance through which the frame rotates about 𝑠̂, in 𝑟𝑎𝑑𝑖𝑎𝑛𝑠
	
	Returns:
		T (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): Final configuration (transformation matrix) ( 4 x 4 )
    """
    Sw = np.array( [S.item(0), S.item(1), S.item(2)] )
    Sv = np.array( [S.item(3), S.item(4), S.item(5)] )

	### CASE 1 (screw pitch is finite) ###
    if abs(1 - norm(Sw)) < 0.05:
        Sv.shape = (3,1)
        skewSw = skew3(Sw)
        e = np.identity(3) + sin(theta)*skewSw + (1 - cos(theta))*(skewSw)**2
        g = np.identity(3)*theta + (1-cos(theta))*skewSw + (theta-sin(theta))*(skewSw**2)
        GSv = g*Sv
        T = np.matrix([ [e.item(0), e.item(1), e.item(2), GSv.item(0)],
                        [e.item(3), e.item(4), e.item(5), GSv.item(1)],
                        [e.item(6), e.item(7), e.item(8), GSv.item(2)],
                        [    0    ,     0    ,     0    ,      1     ]    ])
	
	### CASE 2 (screw pitch is infinite) ###
    else:
        SvTheta = Sv * theta
        T = np.matrix([ [  1  ,  0  ,  0  ,  SvTheta.item(0)  ],
                        [  0  ,  1  ,  0  ,  SvTheta.item(1)  ],
                        [  0  ,  0  ,  1  ,  SvTheta.item(2)  ],
                        [  0  ,  0  ,  0  ,         1         ]    ])
    return T



def RBMmatrixLog(R):
    trR = np.trace(R)

    ### CONDITION 1 ###
    if checkIdentity(R):
        theta = 0
        Sw = None
        print('R = I, theta = 0, omega is undefined.')
    ###################

	### CONDITION 2 ###
    a = 1/2 * (trR - 1)
    theta = acos(a)
    
    b = R - R.T
    c = 2 * sin(theta)
    Sw = (1 / c) * b
	###################

    return theta, Sw


def RBMmatrixLog2(R, p):

	### CONDITION 3 ###
    trR = np.trace(R)
    a = 1/2 * (trR - 1)
    theta = acos(a)
    b = R - R.T
    c = 2 * sin(theta)
    Sw = (1 / c) * b
	###################

    Ginv = (1/theta)*I3 - (1/2)*Sw + ((1/theta) - (1/2)*cot(theta/2))*(Sw**2)

    p.shape = (3,1)
    Sv = Ginv * p
    Sw = unskew(Sw)

    return theta, Sw, Sv


def wrench(r, f):
    """Calculates the wrench given a location vector and force vector.

	Args:
		𝐫 (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): The location vector; the distance from the origin of the frame being acted upon to the point at which the forces act, expressed as [  𝐫x , 𝐫y , 𝐫z  ]
		𝐟 (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): The force vector; the forces occurring at location 𝐫, expressed as [  𝐟x , 𝐟y , 𝐟z  ]

	Returns:
		𝐅 (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): The wrench vector; a combination of the moments and forces acting on the frame being analyzed, expressed as a row vector [  mx , my , mz , 𝐟x , 𝐟y , 𝐟z  ]
    """
    m = np.cross(r, f)
    F = np.array( [m.item(0), m.item(1), m.item(2), f.item(0), f.item(1), f.item(2)] )
    return F


# Returns the Twist (6 vector) given screw pitch, screw axis,
#   rate of rotation, and point on on the screw axis to locate space frame origin
def Twist(h, sHat, thetaDot, q):
	"""Returns the Twist given a screw pitch 𝘩, a screw axis 𝑠̂, rate of rotation 𝜃𝑑𝑜𝑡, and a location vector 𝑞.

	Args:
		𝘩  (𝑖𝑛𝑡/𝑓𝑙𝑜𝑎𝑡): Screw pitch
		𝑠̂  (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Screw axis ( 3-vector, [ x, y, z ] )
		𝜃𝑑𝑜𝑡  (𝑖𝑛𝑡/𝑓𝑙𝑜𝑎𝑡): Rate of rotation about 𝑠̂
		𝑞  (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Vector locating origin of reference frame to 𝑠̂

	Returns:
		𝑉 (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): The Twist ( 1 x 6 ) describing the Screw and rotation
	"""
	cross = np.cross( q, sHat )	# get rid of negative by flipping cross product
	last = cross + h*sHat
	a = np.array( [sHat.item(0), sHat.item(1), sHat.item(2), last.item(0), last.item(1), last.item(2)] )
	V = a*thetaDot
	# print('\nDivide by theta_dot to get S.\n')
	return V



def Twist2(omega, q, h):
    """Returns the Twist given a screw pitch 𝘩, a screw axis 𝑠̂, rate of rotation 𝜃𝑑𝑜𝑡, and a location vector 𝑞.

    Args:
        𝘩 (𝑖𝑛𝑡/𝑓𝑙𝑜𝑎𝑡): Screw pitch
        𝑠̂	(𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Screw axis ( 3-vector, [ x, y, z ] )
        𝜃𝑑𝑜𝑡 (𝑖𝑛𝑡/𝑓𝑙𝑜𝑎𝑡): Rate of rotation about 𝑠̂
        𝑞 (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Vector locating origin of reference frame to 𝑠̂

    Returns:
        𝑉 (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): The Twist ( 1 x 6 ) describing the screw and rotation
    """
    sHat = omega / norm(omega)
    thetaDot = norm(omega)
    cross = np.cross( q, sHat )
    last = cross + h*sHat
    S = np.array( [sHat.item(0), sHat.item(1), sHat.item(2), last.item(0), last.item(1), last.item(2)] )
    V = S*thetaDot
    return V



def Screw(V):
	"""Returns the Screw given a Twist

	Args:
		V (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Twist ( 6 x 6 )

	Returns:
		[Screw, 𝑞, 𝑠̂, 𝘩]: List of values ( Screw and its parameters )
	"""
	thetaDot = norm(V[0:3])
	if thetaDot == 0: thetaDot = norm(V[3:])
	S = V/thetaDot
	S_w = np.array( [S.item(0), S.item(1), S.item(2)] )
	S_v = np.array( [S.item(3), S.item(4), S.item(5)] )
	h = S_w.T * S_v
	s_hat = S_w
	S_vr = S_v - h*s_hat
	q = ( np.cross(s_hat, S_vr) ) / np.dot( -1*s_hat, -1*s_hat )

	vals = [S, q, s_hat, h]
	return vals



def Screw2(omega, q, h):
	"""Returns the Screw given an angular velocity, location/position vector, and screw pitch

	Args:
		omega (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Angular velocity about the Screw axis 𝑠̂
		q (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Location/position vector from reference origin to the screw axis
		h (𝑖𝑛𝑡): Screw pitch

	Returns:
		S (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): The corresponding Screw
	"""
	sHat = omega / norm(omega)
	thetaDot = norm(omega)
	cross = np.cross( q, sHat )
	last = cross + h*sHat
	S = np.array( [sHat.item(0), sHat.item(1), sHat.item(2), last.item(0), last.item(1), last.item(2)] )
	return S


# returns the screw S given location/position vector, screw axis direction, and screw pitch
def Screw3(q, s, h):
	"""Returns the Screw given an location/position vector, screw axis, and screw pitch

	Args:
		q (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Location/position vector from reference origin to the screw axis
		𝑠̂ (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Screw axis ( 3-vector, [ x, y, z ] )
		h (𝑖𝑛𝑡): Screw pitch

	Returns:
		S (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): The corresponding Screw
	"""
	if norm(s) > 1:
		s = s / norm(s)
	Sw = s
	Sv = np.cross( -1*s, q ) + h * s    # angular component
	cross = np.cross(-1*s, q)
	last = cross + h*s
	S = np.array( [s.item(0), s.item(1), s.item(2), last.item(0), last.item(1), last.item(2)] )
	return S


# returns the Screw given the two components Sw and Sv
def Screw4(Sw, Sv):
    """Returns the Screw given the two components Sw and Sv

    Args:
		Sw (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Angular velocity component
		Sv (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Linear velocity component

    Returns:
		S (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Screw ( 1 x 6 )
    """
    S = np.array( [Sw.item(0), Sw.item(1), Sw.item(2), Sv.item(0), Sv.item(1), Sv.item(2)] )
    return S


# T = f.transform(Rsb, ps)
## Returns the transformation matrix given a frame R and a vector p
### Note p must be a numpy array; ex: p = np.array( [x, y, z] )
def Transform(R, p):
	"""Generates the transformation matrix describing a particular rotation and translation

	Args:
		R (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): Rotation frame ( 3 x 3 )
		p (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Vector from origin-to-origin ( 1 x 3 )

	Returns:
		T (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): Transformation matrix ( 4 x 4 )
	
	Example:
		T = f.transform(Rsb, ps)
	"""
	a = matrix([ [1,0,0,p.item(0)],
				 [0,1,0,p.item(1)],
				 [0,0,1,p.item(2)],
				 [0,0,0,    1    ] ])
	b = matrix([ [R.item(0),R.item(1),R.item(2),0],
				 [R.item(3),R.item(4),R.item(5),0],
				 [R.item(6),R.item(7),R.item(8),0],
				 [    0    ,    0    ,    0    ,1] 	])
	T = matmul(a,b)
	T = np.where( matrix.round(T,3) == 0, 0, T )	# gets rid of negative signs in front of zeros
	return T



# AdT = f.adjoint(Rsb, ps)
## Finds the adjoint of a transformation matrix T given a frame R and a vector p
### Note p must be a numpy array; ex: p = np.array( [x, y, z] )
def Adjoint(R, p):
	"""Finds the adjoint of a transformation matrix given a frame and a vector -- uses the frame and vector to construct transformation matrix first

	Args:
		R (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): Rotation frame ( 3 x 3 )

		p (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Vector from origin-to-origin ( 1 x 3 )

	Returns:
		𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥: Adjoint ( 6 x 6 )
	
	Example:
		AdT = f.Adjoint(Rsb, ps)
	"""
	pSkew = skew3(p)
	pR = matmul(pSkew,R)
	AdT = matrix([   [ R.item(0), R.item(1), R.item(2),    0    ,    0    ,    0    ],
				   	 [ R.item(3), R.item(4), R.item(5),    0    ,    0    ,    0    ],
				   	 [ R.item(6), R.item(7), R.item(8),    0    ,    0    ,    0    ],
				   	 [pR.item(0),pR.item(1),pR.item(2),R.item(0),R.item(1),R.item(2)],
				   	 [pR.item(3),pR.item(4),pR.item(5),R.item(3),R.item(4),R.item(5)],
				   	 [pR.item(6),pR.item(7),pR.item(8),R.item(6),R.item(7),R.item(8)]  	])
	return AdT



def Adjoint2(T):
	"""Computes the adjoint of a transformation matrix

	Args:
		T (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): Transformation matrix ( 4 x 4 )

	Returns:
		𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥: Adjoint(T) ( 6 x 6 )
	
	Example:
		>>> AdT = f.Adjoint2(T)
	"""
	R = np.matrix([  [T.item(0), T.item(1), T.item(2)],
					 [T.item(4), T.item(5), T.item(6)],
					 [T.item(8), T.item(9), T.item(10)]  ])
	P = np.array( [T.item(3), T.item(7), T.item(11)] )
	pSkew = skew3(P)
	pR = matmul(pSkew,R)
	AdT = matrix([   [ R.item(0), R.item(1), R.item(2),    0    ,    0    ,    0    ],
				   	 [ R.item(3), R.item(4), R.item(5),    0    ,    0    ,    0    ],
				   	 [ R.item(6), R.item(7), R.item(8),    0    ,    0    ,    0    ],
				   	 [pR.item(0),pR.item(1),pR.item(2),R.item(0),R.item(1),R.item(2)],
				   	 [pR.item(3),pR.item(4),pR.item(5),R.item(3),R.item(4),R.item(5)],
				   	 [pR.item(6),pR.item(7),pR.item(8),R.item(6),R.item(7),R.item(8)]  	])
	return AdT




# matrix = f.matrixSubtract(matrixB, matrixA)
## Returns the result of  matrixB - matrixA
def matrixSubtract(matrixB, matrixA):
	"""Returns the result of ** Matrix B minus Matrix A **

	Args:
		matrixB (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): Matrix B
		matrixA (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): Matrix A

	Returns:
		𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥: Resulting difference matrix
	
	Example:
		>>> matrix = f.matrixSubtract(matrixB, matrixA)
	"""
	C = matrix([  [ matrixB.item(0) - matrixA.item(0), matrixB.item(1) - matrixA.item(1), matrixB.item(2) - matrixA.item(2) ],
                  [ matrixB.item(3) - matrixA.item(3), matrixB.item(4) - matrixA.item(4), matrixB.item(5) - matrixA.item(5) ],
                  [ matrixB.item(6) - matrixA.item(6), matrixB.item(7) - matrixA.item(7), matrixB.item(8) - matrixA.item(8) ]   ])
	return C



# unit_vector = unitVector(vector)
## Returns the unit vector of a given vector
def unitVector(vector):
	"""Returns the unit vector of a given vector

	Args:
		vector (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Standard non-unit vector

	Returns:
		𝑛𝑝.𝑎𝑟𝑟𝑎𝑦: Resulting unit vector
	
	Example:
		>>> unit_vector = unitVector(vector)
	"""
	unit_vector = vector / norm(vector)
	return unit_vector



# matrix = buildMatrix(9, [..., ..., ...])
## Matrix builder function -- returns a matrix given its size and a list of values (top-left -> bottom-right)
def buildMatrix(size, values):
	"""Matrix builder function -- returns a matrix given its size and a list of values (top-left -> bottom-right)

	Args:
		size (𝑖𝑛𝑡): Number of matrix elements ( either 4 or 9 )
		values (𝑖𝑛𝑡/𝑓𝑙𝑜𝑎𝑡): List of elements [ ... , ... , ... , etc ]

	Returns:
		𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥: The resulting matrix ( 2 x 2 ) or ( 4 x 4 )
	
	Example:
		>>> matrix = buildMatrix(4, [1, 2, 3, 4])
	"""
	if size == 4:
		matr = matrix([  [values[0],values[1]],
						 [values[2],values[3]]  ])
	elif size == 9:
		matr = matrix([  [values[0],values[1],values[2]],
						 [values[3],values[4],values[5]],
						 [values[6],values[7],values[8]]  ])
	else:
		print('\n\nSomething went wrong.')
		return
	return matr



# Wb = f.wb2(Rsb, Ws)  --  note: Ws must be in skew-symmetric form
## Returns angular velocity vector given rotation frame and Ws, same as wb() but with function parameters instead of console input
def wb2(Rsb, Ws):
	"""Returns angular velocity vector given rotation frame and Ws

	Args:
		Rsb (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): Rotation frame ( 3 x 3 )
		[Ws] (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Angular velocity of frame in {s} ( 3 x 3 ) -- skew-symmetric form

	Returns:
		Wb (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Angular velocity of frame in {b} frame ( 1 x 3 )
	
	Example:
		>>> Wb = f.wb2(Rsb, Ws)
	"""
	# print('\n\t>> wb2() <<\n\t-----------')
	Rsb_inv = inv(Rsb)
	Wb = np.dot(Rsb_inv, Ws)
	return Wb



# Wb = matrixToVector(Rsb, Ws)
## Gets the vector equivalent of the Wb matrix using the formula  Wb = Rsb.T * Ws  , where Ws is the NON skew-symmetric form (row/column vector)
def matrixToVector(Rsb, Ws):
	"""Gets the vector equivalent of the Wb matrix using the formula  Wb = Rsb.T * Ws

	Args:
		Rsb (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): Rotation frame ( 3 x 3 )
		Ws (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Angular velocity of frame in {s} ( 1 x 3 ) -- NON skew-symmetric form

	Returns:
		Wb (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Angular velocity of frame in {b} frame ( 1 x 3 )
	
	Example:
		>>> Wb = matrixToVector(Rsb, Ws)
	"""
	# print('\n\t>> matrixToVector() <<\n\t----------------------')
	Ws.shape = (3, 1)
	RsbT = Rsb.T
	Wb = RsbT * Ws
	return Wb



# Rsb = f.rotate1('x', 30)
## Rotates the space frame, {s}, theta radians about rotationAxis to get {b}
### 2 Inputs:  which axis the rotation was done on, how far it rotated around that axis
#### Note that theta needs to be passed in as degrees, not radians - use np.degrees to convert
def rotate1(rotationAxis, theta):
	"""Rotates the space frame, {s}, 𝜃 radians about rotationAxis to get {b} frame ( single rotation )

	Args:
		rotationAxis (𝑐𝘩𝑎𝑟): The axis about which the frame is rotated ( either '𝑥' , '𝑦' , or '𝑧' )
		theta (𝑖𝑛𝑡/𝑓𝑙𝑜𝑎𝑡): The angle through which the frame is rotated about rotationAxis ( in degrees )

	Returns:
		Rsb (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): The resulting rotation frame
	
	Example:
		>>> Rsb = f.rotate1('x', 30)
	"""
	if rotationAxis == 'x':
		R = matrix([  [1,        0       ,         0       ],
					  [0, cos(rad(theta)), -sin(rad(theta))],
					  [0, sin(rad(theta)),  cos(rad(theta))]  ])
	elif rotationAxis == 'y':
		R = matrix([  [ cos(rad(theta)), 0, sin(rad(theta))],
					  [        0       , 1,        0       ],
					  [-sin(rad(theta)), 0, cos(rad(theta))]  ])
	elif rotationAxis == 'z':
		R = matrix([  [cos(rad(theta)), -sin(rad(theta)), 0],
					  [sin(rad(theta)),  cos(rad(theta)), 0],
					  [       0       ,         0       , 1]  ])
	else:
		print('\n\n** Something went wrong with rotate()')
		return
	return R



# Rsb = f.rotate2(3, ['z', 'x', 'y'], [30, 40, 60])
## Does the same thing as the rotate() function, but does several rotations in a row
### 3 Inputs:  number of rotations to do, the axes the rotations are done on, how far each rotation goes
#### Note that again theta needs to be passed in as degrees, not radians
def rotate2(numRotations, rotationAxes = [], theta = []):
	"""Rotates the space frame, {s}, [𝜃] radians about [rotationAxes] to get {b} frame ( several rotations )

	Args:
		numRotations (𝑖𝑛𝑡): Number of rotations done
		rotationAxes (𝑙𝑖𝑠𝑡 of 𝑐𝘩𝑎𝑟𝑠): The axes about which the frame is rotated ( either '𝑥' , '𝑦' , or '𝑧' )
		theta (𝑙𝑖𝑠𝑡 of 𝑖𝑛𝑡𝑠/𝑓𝑙𝑜𝑎𝑡𝑠): The angles through which the frame is rotated about each axis ( in degrees )

	Returns:
		Rsb (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): The resulting rotation frame
	
	Example:
		>>> Rsb = f.rotate2(3, ['z', 'x', 'y'], [30, 40, 60])
	"""
	R = []													# R is an empty list, it holds the results of each individual rotation
	for i in range (numRotations):													# loop through the following numRotations times
		if rotationAxes[i] == 'x':														# check if x was the rotation axis
			R.append(matrix([  [1,        0       ,             0         ],			# if it is, create the new frame and add it to R list
					  		   [0, cos(rad(theta[i])), -sin(rad(theta[i]))],
					  		   [0, sin(rad(theta[i])),  cos(rad(theta[i]))]  ]))
		elif rotationAxes[i] == 'y':													# check if y was the rotation axis
			R.append(matrix([  [ cos(rad(theta[i])), 0, sin(rad(theta[i]))],			# if it is, create the new frame and add it to R list
					  		   [          0        , 1,         0         ],
					  		   [-sin(rad(theta[i])), 0, cos(rad(theta[i]))]  ]))
		elif rotationAxes[i] == 'z':													# check if z was the rotation axis
			R.append(matrix([  [ cos(rad(theta[i])), -sin(rad(theta[i])), 0],			# if it is, create the new frame and add it to R list
					  		   [ sin(rad(theta[i])),  cos(rad(theta[i])), 0],
					  		   [          0        ,            0       , 1]  ]))
		else:
			print('\n\nSomething went wrong.')				# if none of the above ran, print error message
			return
	if numRotations < 3:
		Rsb = np.dot(R[0], R[1])				# if the number of rotations was less than 3, then the final frame is just rotation frame 1 * rotation frame 2
	else:
		for i in range (numRotations):			# if the number of rotations was >= 3, multiply all the frames together one by one
			if i == 0 or i == 1:
				Rsb = np.dot(R[0], R[1])
			else:
				Rsb = np.dot(Rsb, R[i])
	return Rsb



# Ws = f.skew2([x,y,z])
## Skew-symmetric function -- returns the skew-symmetric matrix of a column vector (via arguments)
def skew2(vector = []):
	"""Skew-symmetric function -- returns the skew-symmetric matrix of a column/row vector

	Args:
		vector (𝑙𝑖𝑠𝑡 of 𝑖𝑛𝑡𝑠/𝑓𝑙𝑜𝑎𝑡𝑠): Elements of vector as a 𝑙𝑖𝑠𝑡

	Returns:
		𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥: The resulting skew-symmetric matrix ( 3 x 3 )
	
	Example:
		>>> Ws = f.skew2([x,y,z])
	"""
	matrix = matrix([  [     0      , -1*vector[2] ,   vector[1]  ],
					   [ vector[2]  ,       0      , -1*vector[0] ],
					   [-1*vector[1],   vector[0]  ,       0      ]  ])
	return matrix



# Ws = f.skew3(v)
## Skew-symmetric function -- returns the skew-symmetric form (3x3) of a vector
### Note that the vector must be a numpy array; ex: v = np.array( [x, y, z] )
def skew3(vector):
	"""Skew-symmetric function -- returns the skew-symmetric matrix of a column/row vector

	Args:
		vector (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Vector to be skewed ( 1 x 3 )

	Returns:
		𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥: The resulting skew-symmetric matrix ( 3 x 3 )
	
	Example:
		>>> Ws = f.skew3(v)
	"""
	matrix_ = matrix([  [        0        , -1*vector.item(2),   vector.item(1) ],
						[  vector.item(2) ,         0        , -1*vector.item(0)],
						[-1*vector.item(1),   vector.item(0) ,         0        ]  ])
	return matrix_



# v = f.unskew(Ws)
## Vector rebuilder function -- pulls out the x,y,z components of the skew-symmetric matrix and returns the 3 vector
### 1 Input:  any skew-symmetric matrix
def unskew(skewMatrix):
	"""Vector rebuilder function -- pulls out the x , y , z components of the skew-symmetric matrix and returns the 3-vector ( 1 x 3 )

	Args:
		skewMatrix (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): The skew-symmetric matrix to be "unskewed" ( 3 x 3 )

	Returns:
		𝑛𝑝.𝑎𝑟𝑟𝑎𝑦: The resulting column/row vector ( 1 x 3 )
	
	Example:
		>>> v = f.unskew(Ws)
	"""
	x = skewMatrix.item(7)
	y = skewMatrix.item(2)
	z = skewMatrix.item(3)
	vector = np.array( [x, y, z] )
	# colVector.shape = (3,1)
	return vector



# vectors = f.componentVectors3(Rsb)
## Gets the component vectors of a 3x3 matrix
def componentVectors3(matrix):
	vector1 = np.array( [matrix.item(0), matrix.item(3), matrix.item(6)] )
	vector2 = np.array( [matrix.item(1), matrix.item(4), matrix.item(7)] )
	vector3 = np.array( [matrix.item(2), matrix.item(5), matrix.item(8)] )
	vectors = [vector1, vector2, vector3]
	return vectors



# vectors = f.componentVectors3_unit(Rsb)
## Gets the component vectors of a 3x3 matrix as unit vectors
def componentVectors3_unit(matrix):
	vector1 = np.array( [matrix.item(0), matrix.item(3), matrix.item(6)] )
	vector2 = np.array( [matrix.item(1), matrix.item(4), matrix.item(7)] )
	vector3 = np.array( [matrix.item(2), matrix.item(5), matrix.item(8)] )
	vector1u = vector1 / norm(vector1)
	vector2u = vector2 / norm(vector2)
	vector3u = vector3 / norm(vector3)
	unit_vectors = [vector1u, vector2u, vector3u]
	return unit_vectors



# R = f.rodrigues(theta, Ws)
## Rodrigues formula  --  R = I + sin(theta)[w] + (1 - cos(theta)[w]^2)
### 2 Inputs:  an angle theta, and the vector that theta corresponds to
#### Note that the vector (Ws) needs to be passed in in its skew-symmetric form
def rodrigues(theta, Ws):
	"""Rodrigues formula  --  R = I + sin(𝜃)*[𝜔] + (1 - cos(𝜃)*[𝜔]²)

	Args:
		theta (𝑖𝑛𝑡/𝑓𝑙𝑜𝑎𝑡): Angle of rotation ( in degrees )
		Ws (𝑛𝑝.𝑎𝑟𝑟𝑎𝑦): Vector ( 1 x 3 )

	Returns:
		R (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): The corresponding frame
	
	Example:
		>>> R = f.rodrigues(theta, Ws)
	"""
	theta = np.radians(theta)
	I = np.identity(3)
	Ws2 = matmul(Ws, Ws)
	R = I + sin(theta)*Ws + (1 - cos(theta))*(Ws2)
	return R


# print( checkIdentity(R) )
# Checks if a given matrix is an identity matrix -- returns True or False
#   (has some tolerance built in for imprecise floating point numbers) (used in matrixLog())
def checkIdentity(R):
	a_ = R.item(0)
	e_ = R.item(4)
	i_ = R.item(8)
	a = True  if  abs(1 - a_)<0.001  else  False
	e = True  if  abs(1 - e_)<0.001  else  False
	i = True  if  abs(1 - i_)<0.001  else  False
	ones = True  if   a and e and i  else  False

	b_ = R.item(1)
	c_ = R.item(2)
	d_ = R.item(3)
	f_ = R.item(5)
	g_ = R.item(6)
	h_ = R.item(7)
	b = True  if  abs(0 - b_)<0.001  else  False
	c = True  if  abs(0 - c_)<0.001  else  False
	d = True  if  abs(0 - d_)<0.001  else  False
	f = True  if  abs(0 - f_)<0.001  else  False
	g = True  if  abs(0 - g_)<0.001  else  False
	h = True  if  abs(0 - h_)<0.001  else  False
	zeros = True  if   b and c and d and f and g and h  else  False

	if ones and zeros:
		return True
	else:
		return False



# wHat = matrixLog(R)
## Returns the angular velocity vector given some rotation frame R (also prints theta and the exponential coordinates)
def matrixLog(R):
	"""Returns the angular velocity vector given some rotation frame R (also prints theta and the exponential coordinates)

	Args:
		R (𝑛𝑝.𝑚𝑎𝑡𝑟𝑖𝑥): The initial frame

	Returns:
		𝜔Hat and 𝜃 ( 𝑙𝑖𝑠𝑡 [ 𝑛𝑝.𝑎𝑟𝑟𝑎𝑦 , 𝑖𝑛𝑡/𝑓𝑙𝑜𝑎𝑡 ] ): The corresponding angular velocity vector and the angle through which it rotated
	
	Example:
		>>> wHat = matrixLog(R)
	"""
	print('\n\t>> matrixLog() <<\n\t-----------------')
	if checkIdentity(R):
		print('\n>> R is the identity matrix  --  wHat is undefined. Returning wHat as a 3vector with zeros.\n\n')
		wHat = np.array( [0, 0, 0] )
		return wHat

	else:
		trace_R = np.trace(R)

		if round(trace_R, 3) + 1 == 0:
			print('\n>> trace(R) = -1  --  Theta = pi.')
			print('>> wHat will be returned as a list of 3 vectors. (note: if wHat is a solution, then so is -wHat)')
			theta = pi
			theta2 = deg(theta)
			firstTerm1 = 1 / (sqrt( 2 * (1 + R.item(8)) ))
			firstTerm2 = 1 / (sqrt( 2 * (1 + R.item(4)) ))
			firstTerm3 = 1 / (sqrt( 2 * (1 + R.item(0)) ))
			vector1 = np.array( [R.item(2), R.item(5), 1 + R.item(8)] )
			vector2 = np.array( [R.item(1), 1 + R.item(4), R.item(7)] )
			vector3 = np.array( [1 + R.item(0), R.item(3), R.item(6)] )
			wHat1 = firstTerm1 * vector1
			wHat2 = firstTerm2 * vector2
			wHat3 = firstTerm3 * vector3
			expCoordinates1 = wHat1 * theta
			expCoordinates2 = wHat2 * theta
			expCoordinates3 = wHat3 * theta
			print('\n------------------------------------------------------------')
			print('\nExponential coordinates =\n')
			print('\t' + str(expCoordinates1) + '\n   or')
			print('\t' + str(expCoordinates2) + '\n   or')
			print('\t' + str(expCoordinates3))
			print('\n\nwHat =\n')
			print('\t' + str(wHat1) + '\n   or')
			print('\t' + str(wHat2) + '\n   or')
			print('\t' + str(wHat3))
			print('\n\nTheta = ' + str(theta) + ' radians,   ' + str(np.degrees(theta)) + ' degrees\n')
			print('------------------------------------------------------------\n\n\n')
			wHat = [wHat1, wHat2, wHat3]
			results = [wHat, theta2]
			return results

		else:
			theta = np.arccos( (1/2) * (trace_R - 1) )
			theta2 = np.degrees(theta)
			wHat_skew = (  ( 1/(2 * sin(theta)) ) * ( R - R.T ) )
			wHat = unskew(wHat_skew)
			expCoordinates = wHat * theta
			print('\n------------------------------------')
			print('\nExp. coordinates =')
			print('\t' + str(np.around(expCoordinates,5)))
			print('\nwHat =')
			print('\t' + str(np.around(wHat,5)))
			print('\nTheta = ')
			print('\t' + str(np.round(theta,5)) + ' radians')
			print('\t' + str(np.round(np.degrees(theta),5)) + ' degrees\n')
			print('------------------------------------\n\n\n')

			results = [wHat, theta2]	# theta2 is the angle in degrees
			return results



#
# Grubler function -- calculates DOF given input of m, N, J, and joints (either as a list of strings or ints)
def grubler(M, N, J, joints = [], *args):
	"""Grubler function -- calculates DoF given input of m, N, J, and joints (either as a list of strings or ints)

	Args:
		M (𝑠𝑡𝑟/𝑖𝑛𝑡): The spatial geometry DoF ( either '2d' / '3d' or '3' / '6' )
		N (𝑖𝑛𝑡): Total number of joints ( including ground )
		J (_type_): _description_
		joints (𝑙𝑖𝑠𝑡 of 𝑠𝑡𝑟 or 𝑖𝑛𝑡): The joints involved ( either by name or by DoF )

	Returns:
		_type_: _description_
	"""
	fi = []
	sum_fi = 0

	# Check formatting of joints list, handle whichever is given
	if type(joints[0]) is str:
		for i in range(len(joints)):
			fi.append(jointDOF[joints[i]])
	elif type(joints[0]) is int:
		for i in range(len(joints)):
			fi.append(joints[i])
	else:
		print('Improperly formatted joints argument.')
		return

	# Sum freedom of each joint
	for i in range(len(fi)):
		sum_fi += fi[i]
	
	# Check formatting of m, handle whichever is given
	if type(M) is str:
		M = m[M]
	elif type(M) is int:
		pass
	else:
		print('Invalid input m.')
		return
	
	# Calculate degrees of freedom and return it
	DOF = M*(N - 1 - J) + sum_fi
	return DOF



# Rsb = f.rotate()
## Matrix rotation function -- returns Rsb; requires manual input after calling, no arguments passed
def rotate():
	print('\n\n--------------------- ROTATE ---------------------')
	n = int(input('\nHow many rotations?:  '))
	if n > 1:
		R = []
		axis = []
		angle = []

		print(' ')
		for i in range (n):
			axis.append(input('   Enter' + dic[i] + 'rotation axis:\t'))
		print(' ')
		for i in range (n):
			angle.append(rad(float(input('   Enter' + dic[i] + 'rotation angle:\t'))))

		for i in range (n):
			if axis[i] == 'x':
				R.append(matrix([  [1, 0, 0], [0, cos(angle[i]), -sin(angle[i])], [0, sin(angle[i]), cos(angle[i])]  ]))
			elif axis[i] == 'y':
				R.append(matrix([  [cos(angle[i]), 0, sin(angle[i])], [0, 1, 0], [-sin(angle[i]), 0, cos(angle[i])]  ]))
			elif axis[i] == 'z':
				R.append(matrix([  [cos(angle[i]), -sin(angle[i]), 0], [sin(angle[i]), cos(angle[i]), 0], [0, 0, 1]  ]))
			else:
				print('\n\nSomething went wrong.')
				return
		if n < 3:
			Rsb = np.dot(R[0], R[1])
		else:
			for i in range (n):
				if i == 0 or i == 1:
					Rsb = np.dot(R[0], R[1])
				else:
					Rsb = np.dot(Rsb, R[i])
		print('\n\nRsb = ')
		print(Rsb)
		print(' ')
		print('--------------------------------------------------\n')
		return Rsb

	else:
		size = int(input('\nEnter size of the intial frame\'s matrix:  '))
		values = []

		print('\nFrom top-left to bottom-right:\n')
		for i in range (size):
			if i == 6:
				values.append(int(input('   Enter' + dic[i] + 'matrix element: ')))
			else:
				values.append(int(input('   Enter' + dic[i] + 'matrix element:\t ')))

		axis = input('\n   Enter the rotation axis:   ')
		angle = rad(float(input('   Enter the rotation angle:  ')))

		if axis == 'x':
			R = matrix([  [1, 0, 0], [0, cos(angle), -sin(angle)], [0, sin(angle), cos(angle)]  ])
		elif axis == 'y':
			R = matrix([  [cos(angle), 0, sin(angle)], [0, 1, 0], [-sin(angle), 0, cos(angle)]  ])
		elif axis == 'z':
			R = matrix([  [cos(angle), -sin(angle), 0], [sin(angle), cos(angle), 0], [0, 0, 1]  ])
		else:
			print('\n\nSomething went wrong.')
			return

		Rinitial = buildMatrix(size, values)

		Rsb = np.dot(Rinitial, R)

		print('\n\nRsb = ')
		print(Rsb)
		print(' ')
		print('--------------------------------------------------\n')
		return Rsb



# Ws = f.skew()
## Skew-symmetric function -- returns the skew-symmetric matrix of a column vector
def skew():
	print('\n\n---------------------- SKEW ----------------------\n')
	vector = []
	for i in range (3):
		vector.append(float(input('Enter' + dic[i] + 'vector component:\t')))

	matrix = matrix([  [0, -1*vector[2], vector[1]], [vector[2], 0, -1*vector[0]], [-1*vector[1], vector[0], 0]  ])

	print('\nSkew-symmetric matrix of vector (' + str(vector[0]) + ', ' + str(vector[1]) + ', ' + str(vector[2]) + ') =')
	print(matrix)
	print(' ')
	print('--------------------------------------------------\n')
	return matrix



# Wb = f.wb()
## Combination of rotate() and skew() to get Wb -- returns Wb
def wb():
	Rsb = rotate()
	Rsb_inv = inv(Rsb)
	Ws = skew()
	Wb = np.dot(Rsb_inv, Ws)
	return Wb



# deltaVector = vectorDelta(Vi, Vf)
## Returns a new vector holding the difference in x, y, and z components between initial and final vector (used for connecting vectors in animated plots)
def vectorDelta(initialVector, finalVector):
	x = finalVector.item(0) - initialVector.item(0)
	y = finalVector.item(1) - initialVector.item(1)
	z = finalVector.item(2) - initialVector.item(2)

	deltaVector = np.array([  [x, y, z]  ])
	return deltaVector





m = {
	'2d': 3,
	'3d': 6
}
jointDOF = {
	'h': 1,
	'r': 1,
	'p': 1,
	'u': 2,
	's': 3,
	'c': 2,
	'helical': 1,
	'revolute': 1,
	'prismatic': 1,
	'universal': 2,
	'spherical': 3,
	'cylindrical': 2
}
dic = {0: ' first ', 1: ' second ', 2: ' third ', 3: ' fourth ', 4: ' fifth ', 5: ' sixth ', 6: ' seventh ', 7: ' eighth ', 8: ' ninth '}