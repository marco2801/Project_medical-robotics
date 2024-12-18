# first try of implementation of the alghorithm of the paper that you send us


import numpy as np
from scipy.optimize import minimize
#bfgs alghoritm for optimization
beta_opt=np.pi/2 # entry and exit angle ideal
ein_opt=0 # error in the entry point
eout_opt=0 # error in the exit point
sn_opt=0 # needle-wound symmetry
dc=25 #array of needle diameters
an=[0.25,3/8,0.5,5/8] #array of needle shapes
lio=0.05 #number for the wound lenght
depth_ideal=lio/2
lins=0.03 #minimum portion of the needle that can be grasped by the instrument
hti=0.05 # avoid undesidered tissue-needle contact
lambdas=[1,1,1,1,1,1] #weighting factors for optimization
gamma=np.pi/3 #wound angle
s0=0
l0=10 #initial guess
ww=5 # wound width in mm
#lio:y position of needle center
#s0: x position of needle center
#t: distance between entry point and edge of the wound
#ww: width of the wound

# compute the suture parameters using the equations in the paper
def compute_suture_parameters(gamma, s0, l0, ww, dc, lio):
    # Check if the input variables are not lists
    if isinstance(s0, list) or isinstance(l0, list) or isinstance(ww, list) or isinstance(dc, list) or isinstance(lio, list):
        raise ValueError("Input parameters must be numeric, not lists.")

    alpha_1 = np.arcsin(2 * np.sin(gamma / 2) / dc * (lio - np.tan((np.pi - gamma) / 2)) * (lio / 2 + s0))
    alpha_2 = np.arcsin(2 * np.sin(gamma / 2) / dc * (lio - np.tan((np.pi - gamma) / 2)) * (lio / 2 - s0))

    ein = (-dc / 2 * np.cos(alpha_2 + (np.pi + gamma) / 2) + lio / 2 - s0) / (np.cos((np.pi - gamma) / 2))
    eout = (-dc / 2 * np.cos(alpha_1 + (np.pi + gamma) / 2) + lio / 2 + s0) / (np.cos((np.pi - gamma) / 2))
    symmetry = abs(s0)
    t = (lio - ww) / (2 * np.cos((np.pi - gamma) / 2))
    depth = abs(-dc / 2 + l0 - t * np.sin((np.pi - gamma) / 2))
    beta_out = beta_opt + alpha_1
    beta_in = beta_opt + alpha_2
    return [ein, eout, symmetry, depth, beta_out, beta_in]

# try to implement the costratints
def constraints(ww, t, ein, gamma, l0, alpha_1, alpha_2):
    con = []
    # bite time costraint
    qy = np.sin(2 * np.pi * an[1]) * (ww / 2 + (t - ein) * np.cos((np.pi - gamma) / 2)) + np.cos(2 * np.pi * an[1]) * (ein * np.sin((np.pi - gamma) / 2) - l0) - l0
    con.append({'type': 'ineq', 'fun': lambda x: qy - t * np.cos((np.pi - gamma) / 2) - hti})
    #switching time
    lg = (np.pi * an[1] * dc - dc / 2 * (gamma - alpha_1 - alpha_2)) / 2
    con.append({'type': 'ineq', 'fun': lambda x: lg - lins})
    iaoa = dc * np.sin((gamma - alpha_1 - alpha_2) / 2)
    con.append({'type': 'ineq', 'fun': lambda x: iaoa - ww})
    gmy = -dc / 2 + l0
    factor = t * np.sin((np.pi - gamma) / 2)

    con.append({'type': 'ineq', 'fun': lambda x: factor - gmy})

    #extraction time
    con.append({'type': 'ineq', 'fun': lambda x: gmy - factor - hti})
    return con


# try to implement the cost function normalized
def function_J(lambdas, gamma, lio, ww, l0):
    [error_input, error_output, Wound_symmetry, needle_depth, out_angle, in_angle]= compute_suture_parameters(gamma, s0, l0, ww,dc,lio)
    ideal_values = np.array([0, 0, 0, lio/2, np.pi/2, np.pi/2],dtype=float)

    final_values= np.array([error_input, error_output, Wound_symmetry, needle_depth, out_angle, in_angle],dtype=float)

    delt=np.array(len(final_values))
    for i in range(len(final_values)):
        delt[i] =abs( [final_values[i] - ideal_values[i]] )
    J_norm = 0
    #dmax = np.max(delt)
    #dmin = np.min(delt)
    #find the maximum and minimum the old fashioned way
    dmax=0
    dmin=100000.0
    for i in range(len(delt)):
        if delt[i]>=dmax:
            dmax=delt[i]
        if delt[i]<=dmin:
            dmin=delt[i]
    for i in range(len(final_values)):
        J_norm += (lambdas[i] * (delt[i] - dmin) / (dmax - dmin)) / lambdas[i]
    return J_norm


# tried to implement the optimization part, as you can see i think the ideal values are to be used
# but i don't really know how and where to use them
def optimization(ideal_values,lambdas):
    alpha_1= np.arcsin(2*np.sin(gamma/2)/dc*(lio-np.tan((np.pi-gamma)/2))*(lio/2+s0))
    alpha_2=np.arcsin(2*np.sin(gamma/2)/dc*(lio-np.tan((np.pi-gamma)/2))*(lio/2-s0))
    initial_guess = [dc, an[1], s0, l0]
    bounds = [(0, None), (0.25, 0.75), (None, None), (None, None)]
    result = minimize(function_J, initial_guess, args=(lambdas, gamma, lio, ww), bounds=bounds, constraints=constraints(ww, 0.5, 0.1, gamma, l0, alpha_1, alpha_2))
    return result

ideal_values = [0, 0, 0, lio/2, np.pi/2, np.pi/2]
optimized_value = optimization(ideal_values, lambdas)
print("Valori ottimizzati:", optimized_value.x)
