import numpy as np
from scipy.optimize import minimize

beta_opt=np.pi/2
ein_opt=0.0
eout_opt=0.0
sn_opt=0.0
dc=25.0
an=np.array([0.25,3/8,0.5,5/8])
lio=0.05
depth_ideal=lio/2.0
lins=0.03
hti=0.05
lambdas=np.array([1,1,1,1,1,1])
gamma=np.pi/3.0
s0=0
l0=10.0
ww=5.0



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

def constraints(ww, t, ein, gamma, l0, alpha_1, alpha_2):
    con = []
    qy = np.sin(2 * np.pi * an[1]) * (ww / 2 + (t - ein) * np.cos((np.pi - gamma) / 2)) + np.cos(2 * np.pi * an[1]) * (ein * np.sin((np.pi - gamma) / 2) - l0) - l0
    con.append({'type': 'ineq', 'fun': lambda x: qy - t * np.cos((np.pi - gamma) / 2) - hti})
    lg = (np.pi * an[1] * dc - dc / 2 * (gamma - alpha_1 - alpha_2)) / 2
    con.append({'type': 'ineq', 'fun': lambda x: lg - lins})
    iaoa = dc * np.sin((gamma - alpha_1 - alpha_2) / 2)
    con.append({'type': 'ineq', 'fun': lambda x: iaoa - ww})
    gmy = -dc / 2 + l0
    factor = t * np.sin((np.pi - gamma) / 2)
    con.append({'type': 'ineq', 'fun': lambda x: factor - gmy})
    con.append({'type': 'ineq', 'fun': lambda x: gmy - factor - hti})
    return con

def function_J(lambdas, gamma, lio, ww, l0):
    [error_input, error_output, Wound_symmetry, needle_depth, out_angle, in_angle]= compute_suture_parameters(gamma, s0, l0, ww,dc,lio)
    final_values = [error_input, error_output, Wound_symmetry, needle_depth, out_angle, in_angle]
    ideal_values = [0, 0, 0, lio/2, np.pi/2, np.pi/2]
    delt = [final_values[i] - ideal_values[i] for i in range(len(final_values))]
    J_norm = 0
    dmax = np.max(delt)
    dmin = np.min(delt)
    for i in range(len(final_values)):
        J_norm += (lambdas[i] * (delt[i] - dmin) / (dmax - dmin)) / lambdas[i]
    return J_norm

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
