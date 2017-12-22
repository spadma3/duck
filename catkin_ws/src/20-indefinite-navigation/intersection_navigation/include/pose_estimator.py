import rospy
import numpy as np

# the function estimate pose(see further down) should be used to get a pose estimate. This function uses the following function EKF_step:

def EKF_step(PoseList, VarianceList, Input, InputList, *Measurement)
		b = 0.05 #wheelbase [m], possible improvement: get value from devel sys id
		k =	0.9 #how to determine this?, kinematics covariance parameter
		#todo: measurement covariance parameters:
		r_x =
		r_y =
		r_theta = 

		#Measurement = [x, y ,theta, ]'

		

		#if there is a measurement
		if len(Measurement) != 0:

			#select pose at time of Measurement origin from poselist, same for input
			t = Measurement[3] #timestamp of Measurement
			n = #delta t

			#find index of past input/state corresponding to the new measurement

			for i in range( len(InputList[0,:]) )

				if InputList[2,i] > t #if input happened after measurement
				return(i)


			x = PoseList[:,i] #PoseList(:,n) #3x1 vector(numpy array)
			P = VarianceList[:,:,i]#intial state variance
			u = Input     # latest input, 2x1 vector vl vr
			
			
			

			#predict mean xp of pose after input by forward kinematics motion model
			xp = x + np.array([ [ ( u[0] + u[1] )/2 * cos( x[2] + ( u[1] - u[0] ) / (2*b) ) 	],
	    						[ ( u[0] + u[1] )/2 * sin( x[2] + ( u[1] - u[0] ) / (2*b) ) 	],
	    						[ ( u[1] - u[0] )/b												] 	])



			# calculate Jacobian of motion model w.r.t state 3x3
			F_x = np.array([	[	1, 0, -sin(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2)	],
			  					[  	0, 1,  cos(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2)	],
			    				[	0, 0,  1													]  ])

			# calculate Jacobian of motion model w.r.t input 3x2

			F_u = np.array([	[	cos(x[2] - (u[0] - u[1])/(2*b))/2 + (sin(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b), cos(x[2] - (u[0] - u[1])/(2*b))/2 - (sin(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b) ],
					    		[	sin(x[2] - (u[0] - u[1])/(2*b))/2 - (cos(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b), sin(x[2] - (u[0] - u[1])/(2*b))/2 + (cos(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b) ],
					    		[	-1/b,                                                                                    1/b 	]	])
					   

			#predict variance Pp of pose after input by forward kinematics

			Q = np.array([	[ k * abs(u[0]), 0 ],
							[ 0, k * abs(u[1]) ]
									])
			Pp = F_x @ P @ np.transpose(F_x) + F_u @ Q @ np.transpose(F_u)

			##measurement update

			#jacobian of measurement model wrt state
			H = np.identity(3)

			# wrt to noise
			M = np.identity(3)

			#measurement covariance
			R = np.diag([r_x, r_y, r_theta])

			#measurement	
			z = Measurement




			#Kalman Gain
			K = Pp @ np.transpose(H) @ np.linalg.inv( H @ Pp @ np.transpose(H) + M @ R @ np.transpose(M) )

			#state after measurement update
			xm = xp + K @ (z - xp)

			Pm = (np.identity(3) - K @ H) @ Pp

			x = xm
			P = Pm



			# prediction update N times until current time

			for t in trange(n ?):
			

				##predict mean xp of pose after input by forward kinematics motion model
				xp = x + np.array([ [ ( u[0] + u[1] )/2 * cos( x[2] + ( u[1] - u[0] ) / (2*b) ) 	],
	    						[ ( u[0] + u[1] )/2 * sin( x[2] + ( u[1] - u[0] ) / (2*b) ) 	],
	    						[ ( u[1] - u[0] )/b												] 	])



				# calculate Jacobian of motion model w.r.t state 3x3
				F_x = np.array([	[	1, 0, -sin(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2)	],
			  					[  	0, 1,  cos(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2)	],
			    				[	0, 0,  1													]  ])

				# calculate Jacobian of motion model w.r.t input 3x2

				F_u = np.array([	[	cos(x[2] - (u[0] - u[1])/(2*b))/2 + (sin(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b), cos(x[2] - (u[0] - u[1])/(2*b))/2 - (sin(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b) ],
					    		[	sin(x[2] - (u[0] - u[1])/(2*b))/2 - (cos(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b), sin(x[2] - (u[0] - u[1])/(2*b))/2 + (cos(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b) ],
					    		[	-1/b,                                                                                    1/b 	]	])
					   

				#predict variance Pp of pose after input by forward kinematics

				Q = np.array([	[ k * abs(u[0]), 0 ],
							[ 0, k * abs(u[1]) ]
									])
				Pp = F_x @ P @ np.transpose(F_x) + F_u @ Q @ np.transpose(F_u)

				x = xp
				P = Pp

			xf = x
			Pf = P

			






	

		#if there is no measurement
		else 

			#rename input arguments
			x = PoseList[:,0] #3x1 vector(numpy array)
			u = Input      #2x1 vector
			P = VarianceList[:,:,i]#intial state variance


			##predict mean xp of pose after input by forward kinematics motion model
			xp = x + np.array([ [ ( u[0] + u[1] )/2 * cos( x[2] + ( u[1] - u[0] ) / (2*b) ) 	],
	    						[ ( u[0] + u[1] )/2 * sin( x[2] + ( u[1] - u[0] ) / (2*b) ) 	],
	    						[ ( u[1] - u[0] )/b												] 	])



			# calculate Jacobian of motion model w.r.t state 3x3
			F_x = np.array([	[	1, 0, -sin(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2)	],
			  					[  	0, 1,  cos(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2)	],
			    				[	0, 0,  1													]  ])

			# calculate Jacobian of motion model w.r.t input 3x2

			F_u = np.array([	[	cos(x[2] - (u[0] - u[1])/(2*b))/2 + (sin(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b), cos(x[2] - (u[0] - u[1])/(2*b))/2 - (sin(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b) ],
					    		[	sin(x[2] - (u[0] - u[1])/(2*b))/2 - (cos(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b), sin(x[2] - (u[0] - u[1])/(2*b))/2 + (cos(x[2] - (u[0] - u[1])/(2*b))*(u[0]/2 + u[1]/2))/(2*b) ],
					    		[	-1/b,                                                                                    1/b 	]	])
					   

			#predict variance Pp of pose after input by forward kinematics

			Q = np.array([	[ k * abs(u[0]), 0 ],
							[ 0, k * abs(u[1]) ]
									])

			Pp = F_x @ P @ np.transpose(F_x) + F_u @ Q @ np.transpose(F_u)

			xf = xp
			Pf = Pp



		#add xf and Pf and u to list of x and Ps and u and shorten those lists


		return(xf,Pf) 


def EstimatePose(InitalPose=False, InitialVariance=False, Input, Inputlength, Measurement=False)

	
	Input = np.array([ 	[Input.vel_left * Inputlength]   #wheel distance = velocity * input duration
						[Input.vel_right * Inputlength]
						[Input.header.stamp]	]) #make np array

	
	InputList = np.array([ [Input, InputList]	]) #add new inputs to list


	if InitialPose != False
		Poselist = np.array([ 	[InitalPose] 
								[Input[2]]			]) # enter initialpose with timestamp into poselist

	if InitialVariance != False
		Variancelist = np.array([ 	[InitalPose] 			#same with variance
									[Input[2]]		])




	 
#if there is a measurement
	if Measurement != False
		
		xandP = EKF_step(PoseList, VarianceList, Input, InputList, Measurement)

	else
	
		xandP =EKF_step(PoseList, VarianceList, Input, InputList)




#update lists with new values
	x = np.array([ 	[xandP[:,0]]
					[Input[2]]		]) #x equals state from ekf step with the timestamp of corresponding input appended

	PoseList = np.array([ [x, PoseList]	]) #add new inputs to list



	P = np.array([ 	[xandP[:,1]]
					[Input[2]]		]) 

	VarianceList = np.array([ [P, VarianceList]	])

	


	x= xandP[:,0]
	
	return (x)	

