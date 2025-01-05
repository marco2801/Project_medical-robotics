import numpy as np
from scipy.optimize import brute
from scipy.optimize import fmin
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# input costants (geoemtric parameters of the suture)
lio = 16 # length between actual and desired entry/exit points (mm) -- 3 times ww in the paper
ww = 5.5  # wound width, space between vessel (mm)
lins = 10  # minimum grasping needle length (mm)
hti = 8  # Input to stop unwanted needle-tissue contact at the end of the suture (mm)
gamma = np.pi*4/5 # angle between vessels
lambda_weights = [1, 1, 1, 1, 1, 1]  # weights for suture parameters (tunable)
an_values = [1/4, 3/8, 1/2, 5/8]  # discrete values of needle shape

# t = distance between desired entry (Id) and input edge of wound (Ei) -- constant
t = (lio - ww) / (2 * np.cos((np.pi - gamma) / 2)) # = 5.52mm (gamma =4pi/5, ww=5.5, lio=16)

# COST FUNCTION (without constraints)
def cost_function(needle_vars, *args):
    s0, l0, dc = needle_vars            
    gamma, lio, ww, lambda_weights, an = args

    # Suture parameters computation
    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * (l0 - np.tan((np.pi - gamma)/2) * (lio/2 + s0)),
        -1, 1
    ))    # output angle between tissue surface and the needle center (CLIP does make sense?)
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * (l0 - np.tan((np.pi - gamma)/2) * (lio/2 - s0)),
        -1, 1
    ))    # input angle between tissue surface and the needle center (CLIP does make sense?)

    beta_in = np.pi/2 + alpha_2 #entry angle
    beta_out = np.pi/2 + alpha_1 #exit angle

    dh = abs(-dc/2 + l0 - t * np.sin((np.pi-gamma)/2))     #suture depth [mm]
    sn = abs(s0)    # simmetry [mm]

    ein = (-dc / 2 * np.cos(alpha_2 + (np.pi - gamma) / 2) + lio / 2 - s0) / (np.cos((np.pi - gamma) / 2))  # entry distance actual-desired [mm]
    eout = (-dc / 2 * np.cos(alpha_1 + (np.pi - gamma) / 2) + lio / 2 + s0) / (np.cos((np.pi - gamma) / 2)) # exit distance actual-desired [mm]

    #DELTA: actual values minus the ideal ones --> [mm] or [rad]
    terms = [beta_in - np.pi / 2, ein, dh - lio / 2, sn, beta_out - np.pi / 2, eout] 
    weighted_terms = [
        lambda_weights[i] * abs(terms[i]) for i in range(len(terms))
    ]
    return sum(weighted_terms)

## CONSTRAINTS DEFINITION

#1) BT = BITE TIME
def bite_time_constraint(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an = args
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * (l0 - np.tan((np.pi - gamma) / 2) * (lio / 2 - s0)),
        -1, 1
    ))
    ein = (-dc / 2 * np.cos(alpha_2 + (np.pi - gamma) / 2) + lio / 2 - s0) / (np.cos((np.pi - gamma) / 2))
    qy = (
        np.sin(2 * np.pi * an) * (ww / 2 + (t - ein) * np.cos((np.pi - gamma) / 2) - s0)
        + np.cos(2 * np.pi * an) * (ein * np.sin((np.pi - gamma) / 2) - l0)
        + l0
    )
    return qy - t * np.sin((np.pi - gamma) / 2) - hti

#2.1) SW = SWITCHING TIME constraint --> possible needle grasping
def switching_time_constraint_1(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an = args

    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * (l0 - np.tan((np.pi - gamma)/2) * (lio/2 + s0)),
        -1, 1
    ))
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * (l0 - np.tan((np.pi - gamma) / 2) * (lio / 2 - s0)),
        -1, 1
    ))
    lg = (np.pi*an*dc - dc/2*(gamma-alpha_1-alpha_2))/2

    return lg-lins

#2.2) SW = SWITCHING TIME constraint --> the needle passes externally to the wound
def switching_time_constraint_2(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an = args

    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * (l0 - np.tan((np.pi - gamma)/2) * (lio/2 + s0)),
        -1, 1
    ))
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * (l0 - np.tan((np.pi - gamma) / 2) * (lio / 2 - s0)),
        -1, 1
    ))
    Ia_Oa = dc*np.sin((gamma-alpha_1-alpha_2)/2)

    return Ia_Oa-ww

#2.3) SW = SWITCHING TIME constraint --> the needle must be inside the tissue
def switching_time_constraint_3(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an = args

    dh_coord = -dc/2 +l0 -t*np.sin((np.pi-gamma)/2)

    return -dh_coord

#2.4) SW = SWITCHING TIME constraint --> the needle enters from one side (it doesn't work now)
def switching_time_constraint_4(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an = args

    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * (l0 - np.tan((np.pi - gamma)/2) * (lio/2 - s0)),
        -1, 1
    ))
    ein = (-dc / 2 * np.cos(alpha_2 + (np.pi - gamma) / 2) + lio / 2 - s0) / (np.cos((np.pi - gamma) / 2))
    Id_Ei = [t*np.cos(np.pi-((np.pi-gamma)/2)),t*np.sin(np.pi-((np.pi-gamma)/2))]
    Id_Ia = [ein*np.cos(np.pi-(np.pi-gamma)/2),ein*np.sin(np.pi-(np.pi-gamma)/2)]

    return 1 #- np.inner(Id_Ia,Id_Ei)/t**2

#2.5) SW = SWITCHING TIME constraint --> the needle exits from the other side (it doesn't work now)
def switching_time_constraint_5(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an = args

    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * (l0 - np.tan((np.pi - gamma)/2) * (lio/2 + s0)),
        -1, 1
    ))
    eout = (-dc / 2 * np.cos(alpha_1 + (np.pi - gamma) / 2) + lio / 2 + s0) / (np.cos((np.pi - gamma) / 2))
    Od_Eo = [t*np.cos((np.pi-gamma)/2), t*np.sin((np.pi-gamma)/2)]
    Od_Oa = [eout*np.cos(np.pi-((np.pi-gamma)/2)), eout*np.sin(np.pi-((np.pi-gamma)/2))]

    return 1 #- np.inner(Od_Oa,Od_Eo)/t**2

#3) ET = EXTRACTION TIME
def extraction_time_constraint(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an = args

    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * (l0 - np.tan((np.pi - gamma) / 2) * (lio / 2 + s0)),
        -1, 1
    ))
    eout = (-dc / 2 * np.cos(alpha_1 + (np.pi - gamma) / 2) + lio / 2 + s0) / (np.cos((np.pi - gamma) / 2))
    py = (
       -np.sin(2 * np.pi * an) * (-ww / 2 - (t - eout) * np.cos((np.pi - gamma) / 2) - s0)
        + np.cos(2 * np.pi * an) * (eout * np.sin((np.pi - gamma) / 2) - l0)
        + l0
    )

    return py - t * np.sin((np.pi - gamma) / 2) - hti

# Constraints application
def cost_function_brute(needle_vars, gamma, lio, ww, lambda_weights, an):

    # constraints verification giving infinite value to cost function out of constraints
    if not bite_time_constraint(needle_vars, gamma, lio, ww, lambda_weights, an) >= 0:
        return np.inf
    if not switching_time_constraint_1(needle_vars, gamma, lio, ww, lambda_weights, an) >= 0:
        return np.inf
    if not switching_time_constraint_2(needle_vars, gamma, lio, ww, lambda_weights, an) >= 0:
        return np.inf
    if not switching_time_constraint_3(needle_vars, gamma, lio, ww, lambda_weights, an) >= 0:
        return np.inf
    if not switching_time_constraint_4(needle_vars, gamma, lio, ww, lambda_weights, an) >= 0:
        return np.inf
    if not switching_time_constraint_5(needle_vars, gamma, lio, ww, lambda_weights, an) >= 0:
        return np.inf
    if not extraction_time_constraint(needle_vars, gamma, lio, ww, lambda_weights, an) >= 0:
        return np.inf

    return cost_function(needle_vars, gamma, lio, ww, lambda_weights, an)

# Feasible ranges
ranges = [
    (-lio / 2, lio / 2),  # s0
    (0, 2*lio),           # l0
    (10, 77)              # dc
]

Ns = 30  # grid resolution

best_solution = None
best_cost = float('inf')
optimal_solutions = []

for an in an_values:

    result = brute(
        cost_function_brute,
        ranges=ranges,
        args=(gamma, lio, ww, lambda_weights, an),
        full_output=True,
        finish= None,
        Ns=Ns,
        disp = True
    )

    min_cost, min_vars = result[1], result[0]
    optimal_solutions.append((min_cost, min_vars, an))

    if min_cost < best_cost:
        best_cost = min_cost
        best_solution = (min_vars, an)

if best_solution: #The best solution among the an values
    
    optimal_vars, optimal_an = best_solution

    optimal_s0 = optimal_vars[0]
    optimal_l0 = optimal_vars[1] 
    optimal_dc = optimal_vars[2]  

    # optimal results (debugging)
    alpha_1_best = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / optimal_dc * ((optimal_l0 - np.tan((np.pi - gamma) / 2)) * (lio / 2 + optimal_s0)),
        -1, 1
    ))
    alpha_2_best = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / optimal_dc * ((optimal_l0 - np.tan((np.pi - gamma)/2)) * (lio/2 - optimal_s0)),
        -1, 1
    ))
    beta_in_best = np.pi/2 + alpha_2_best
    beta_out_best = np.pi/2 + alpha_1_best
    ein_best = (-optimal_dc / 2 * np.cos(alpha_2_best + (np.pi - gamma) / 2) + lio / 2 - optimal_s0) / (np.cos((np.pi - gamma) / 2))
    eout_best = (-optimal_dc / 2 * np.cos(alpha_1_best + (np.pi - gamma) / 2) + lio / 2 + optimal_s0) / (np.cos((np.pi - gamma) / 2))
    dh_best = abs(-optimal_dc/2 + optimal_l0 - t*np.sin((np.pi-gamma)/2))
    sn_best = abs(optimal_s0)
    lg_best = (np.pi*optimal_an*optimal_dc - optimal_dc/2*(gamma-alpha_1_best-alpha_2_best))/2
    
    print(f"Optimal solution found using brute force: Cost function value C = {best_cost}, s0={optimal_s0:.2f}, "
          f"l0={optimal_l0:.2f}, dc={optimal_dc:.2f}, an={optimal_an:.2f}")
    print(f"Optimal values of suture parameters: beta_in = {beta_in_best:.2f}, beta_out = {beta_out_best:.2f}, e_in = {ein_best:.2f}, "
          f"e_out = {eout_best:.2f}, sn = {sn_best:.2f}, dh = {dh_best:.2f}, alpha_1 = {alpha_1_best:.2f}, "
          f"alpha_2 = {alpha_2_best:.2f}, lg = {lg_best:.2f}")

    # mesh grid
    s0_vals = np.linspace(-lio / 2, lio / 2, 30)  
    l0_vals = np.linspace(0, 2*lio, 30)            
    dc_vals = np.linspace(10, 77, 30)     

    # VISUALIZATION of cost functions
    fig = plt.figure(figsize=(16, 12))

    for idx, an in enumerate(an_values):

        S0, L0, DC = np.meshgrid(s0_vals, l0_vals, dc_vals)
        
        ax = fig.add_subplot(2, 2, idx+1, projection='3d')
        ax.set_title(f"Cost function values (an={an:.2f})")
        ax.set_xlabel('s0')
        ax.set_ylabel('l0')
        ax.set_zlabel('dc')
        
        Costs = np.zeros(S0.shape)
        for i in range(S0.shape[0]):
            for j in range(S0.shape[1]):
                for k in range(S0.shape[2]):
                    s0 = S0[i, j, k]
                    l0 = L0[i, j, k]
                    dc = DC[i, j, k]
                    cost = cost_function_brute(
                        (s0, l0, dc), 
                        gamma, lio, ww, lambda_weights, an)
                    Costs[i, j, k] = cost if np.isfinite(cost) else np.nan
                    #print(f"Processing an={an}, idx={idx}, s0={s0}, l0={l0}, dc={dc}, cost={cost}")
       
        scatter = ax.scatter(S0.flatten(), L0.flatten(), DC.flatten(), c=Costs.flatten(), cmap='viridis')
        fig.colorbar(scatter, ax=ax, shrink=0.5, aspect=10, label="Cost")
    
    plt.tight_layout()
    plt.show()

else:
    print("The brute force optimization did not find a valid solution.")
