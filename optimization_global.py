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

# DELTA_order: beta_in, e_in, dh, sn, beta_out, e_out (IS IT CORRECT? HOW TO DEFINE THE MAX ERRORS?)
# 38.5mm is the max radius of the needle (dc/2)
delta_min = [0, 0, 0, 0, 0, 0]  # least feasible value for the errors between actual and desired results
delta_max = [np.pi/2, 38.5, 38.5-lio/2, lio, np.pi/2, 38.5]  # highest possible errors

# t = distance between desired entry (Id) and input edge of wound (Ei) -- constant
t = (lio - ww) / (2 * np.cos((np.pi - gamma) / 2))

# COST FUNCTION (without constraints)
def cost_function(needle_vars, *args):
    s0, l0, dc = needle_vars            
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    # Suture parameters computation
    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * ((l0 - np.tan((np.pi - gamma)/2)) * (lio/2 + s0)),
        -1, 1
    ))    # output angle between tissue surface and the needle center (CLIP does make sense?)
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * ((l0 - np.tan((np.pi - gamma)/2)) * (lio/2 - s0)),
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
    normalized_terms = [
        (abs(terms[i]) - delta_min[i]) / (delta_max[i] - delta_min[i])
        for i in range(len(terms))
    ]

    # Normalized cost function
    weighted_terms = [
        lambda_weights[i] * normalized_terms[i] for i in range(len(normalized_terms))
    ]
    return sum(weighted_terms)/sum(lambda_weights)

## CONSTRAINTS DEFINITION

#1) BT = BITE TIME
def bite_time_constraint(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * ((l0 - np.tan((np.pi - gamma) / 2)) * (lio / 2 - s0)),
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
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * ((l0 - np.tan((np.pi - gamma)/2)) * (lio/2 + s0)),
        -1, 1
    ))
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * ((l0 - np.tan((np.pi - gamma) / 2)) * (lio / 2 - s0)),
        -1, 1
    ))
    lg = (np.pi*an*dc - dc/2*(gamma-alpha_1-alpha_2))/2

    return lg-lins

#2.2) SW = SWITCHING TIME constraint --> the needle passes externally to the wound
def switching_time_constraint_2(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * ((l0 - np.tan((np.pi - gamma)/2)) * (lio/2 + s0)),
        -1, 1
    ))
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * ((l0 - np.tan((np.pi - gamma) / 2)) * (lio / 2 - s0)),
        -1, 1
    ))
    Ia_Oa = dc*np.sin((gamma-alpha_1-alpha_2)/2)

    return Ia_Oa-ww

#2.3) SW = SWITCHING TIME constraint --> the needle must be inside the tissue
def switching_time_constraint_3(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    dh_coord = -dc/2 +l0 -t*np.sin((np.pi-gamma)/2)

    return -dh_coord

#2.4) SW = SWITCHING TIME constraint --> the needle enters from one side (it doesn't work now)
def switching_time_constraint_4(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * ((l0 - np.tan((np.pi - gamma)/2)) * (lio/2 - s0)),
        -1, 1
    ))
    ein = (-dc / 2 * np.cos(alpha_2 + (np.pi - gamma) / 2) + lio / 2 - s0) / (np.cos((np.pi - gamma) / 2))
    Id_Ei = [t*np.cos(np.pi-((np.pi-gamma)/2)),t*np.sin(np.pi-((np.pi-gamma)/2))]
    Id_Ia = [ein*np.cos(np.pi-(np.pi-gamma)/2),ein*np.sin(np.pi-(np.pi-gamma)/2)]

    return 1 #- np.inner(Id_Ia,Id_Ei)/t**2

#2.5) SW = SWITCHING TIME constraint --> the needle exits from the other side (it doesn't work now)
def switching_time_constraint_5(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * ((l0 - np.tan((np.pi - gamma)/2)) * (lio/2 + s0)),
        -1, 1
    ))
    eout = (-dc / 2 * np.cos(alpha_1 + (np.pi - gamma) / 2) + lio / 2 + s0) / (np.cos((np.pi - gamma) / 2))
    Od_Eo = [t*np.cos((np.pi-gamma)/2), t*np.sin((np.pi-gamma)/2)]
    Od_Oa = [eout*np.cos(np.pi-((np.pi-gamma)/2)), eout*np.sin(np.pi-((np.pi-gamma)/2))]

    return 1 #- np.inner(Od_Oa,Od_Eo)/t**2

#3) ET = EXTRACTION TIME
def extraction_time_constraint(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * ((l0 - np.tan((np.pi - gamma) / 2)) * (lio / 2 + s0)),
        -1, 1
    ))
    eout = (-dc / 2 * np.cos(alpha_1 + (np.pi - gamma) / 2) + lio / 2 + s0) / (np.cos((np.pi - gamma) / 2))
    py = (
       np.sin(2 * np.pi * an) * (-ww / 2 - (t - eout) * np.cos((np.pi - gamma) / 2) - s0)
        + np.cos(2 * np.pi * an) * (eout * np.sin((np.pi - gamma) / 2) - l0)
        + l0
    )

    return py - t * np.sin((np.pi - gamma) / 2) - hti


def cost_function_brute(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max):

    # constraints verification giving infinite value to cost function out of constraints
    if not bite_time_constraint(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf
    if not switching_time_constraint_1(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf
    if not switching_time_constraint_2(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf
    if not switching_time_constraint_3(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf
    if not switching_time_constraint_4(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf
    if not switching_time_constraint_5(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf
    if not extraction_time_constraint(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf

    return cost_function(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max)

# Feasible ranges
ranges = [
    (-lio / 2, lio / 2),  # s0
    (0, 2*lio),           # l0
    (10, 77)              # dc
]

Ns = 20  # grid resolution

best_solution = None
best_cost = float('inf')

for an in an_values:

    result = brute(
        cost_function_brute,
        ranges=ranges,
        args=(gamma, lio, ww, lambda_weights, an, delta_min, delta_max),
        full_output=True,
        finish= fmin,
        Ns=Ns,
        disp = True
    )

    min_cost, min_vars = result[1], result[0]

    if min_cost < best_cost:
        best_cost = min_cost
        best_solution = (min_vars, an)


if best_solution:
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
    s0_vals = np.linspace(-lio / 2, lio / 2, 50)  
    l0_vals = np.linspace(0, 2*lio, 50)            
    dc_vals = np.linspace(10, 77, 50)     

    # VISUALIZATION

    fig = plt.figure(figsize=(16, 12))

    for idx, an in enumerate(an_values):

        S0, L0, DC = np.meshgrid(s0_vals, l0_vals, dc_vals)
        
        ax = fig.add_subplot(2, 2, idx+1, projection='3d')
        ax.set_title(f"Cost Landscape for an={an:.2f}")
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
                        gamma, lio, ww, lambda_weights, an, delta_min, delta_max
                        )
                    Costs[i, j, k] = cost if np.isfinite(cost) else np.nan
                    #print(f"Processing an={an}, idx={idx}, s0={s0}, l0={l0}, dc={dc}, cost={cost}")
       
        scatter = ax.scatter(S0.flatten(), L0.flatten(), DC.flatten(), c=Costs.flatten(), cmap='viridis')
        fig.colorbar(scatter, ax=ax, shrink=0.5, aspect=10, label="Cost")
    
    plt.tight_layout()
    plt.show()

else:
    print("The brute force optimization did not find a valid solution.")

##% NEEDLE PLOTTING (MUST BE CORRECTED)

# rear of the needle (Q)
#qx_best = (
#        np.cos(2 * np.pi * optimal_an) * (ww / 2 + (t - ein_best) * np.cos((np.pi - gamma) / 2) - optimal_s0)
#        - np.sin(2 * np.pi * optimal_an) * (ein_best * np.sin((np.pi - gamma) / 2) - optimal_l0)
#        + optimal_s0
#)
#qy_best = (
#        np.sin(2 * np.pi * optimal_an) * (ww / 2 + (t - ein_best) * np.cos((np.pi - gamma) / 2) - optimal_s0)
#        + np.cos(2 * np.pi * optimal_an) * (ein_best * np.sin((np.pi - gamma) / 2) - optimal_l0)
#        + optimal_l0
#)

#tip of the needle (P)
#px_best = (
#        np.cos(2 * np.pi * optimal_an) * (-ww / 2 - (t - eout_best) * np.cos((np.pi - gamma) / 2) - optimal_s0)
#        - np.sin(2 * np.pi * optimal_an) * (eout_best * np.sin((np.pi - gamma) / 2) - optimal_l0)
#        + optimal_s0
#)
#py_best = (
#        np.sin(2 * np.pi * optimal_an) * (-ww / 2 - (t - eout_best) * np.cos((np.pi - gamma) / 2) - optimal_s0)
#        + np.cos(2 * np.pi * optimal_an) * (eout_best * np.sin((np.pi - gamma) / 2) - optimal_l0)
#        + optimal_l0
#)

radius = optimal_dc / 2
#theta_start = np.arctan2(py_best-optimal_l0,px_best-optimal_s0)
#theta_end = np.arctan2(qy_best-optimal_l0,qx_best-optimal_s0)
#theta = np.linspace(theta_start, theta_end, 100)  # Lower half of the circle

fig, ax = plt.subplots(figsize=(10, 8))

theta = np.linspace(np.pi, 2*np.pi, 100)  # not correct
x_circle = radius * np.cos(theta) + optimal_s0
y_circle = radius * np.sin(theta) + optimal_l0 
ax.plot(x_circle, y_circle, label=f'Needle Geometry (an={optimal_an:.2f}, dc={optimal_dc:.2f})')

ax.plot(optimal_s0, optimal_l0, 'ro', label=f'Optimal Point (s0={optimal_s0:.2f}, l0={optimal_l0:.2f})')
#ax.plot(px_best,py_best,'bo',label=f'Tip of the needle')
#ax.plot(qx_best,qy_best,'ko',label=f'Rear of the needle')
ideal_points = [lio / 2, -lio / 2]
ax.plot(ideal_points, [0, 0], 'go', label=f'Desired Points (±lio/2)', markersize=5)

# vessel geometry
ax.plot([ww / 2, ww / 2], [(lio/2-ww/2)*np.tan((np.pi-gamma)/2), -2 * lio], 'k')
ax.plot([-ww / 2, -ww / 2], [(lio/2-ww/2)*np.tan((np.pi-gamma)/2), -2 * lio], 'k')
Eo = [-ww/2, (lio/2-ww/2)*np.tan((np.pi-gamma)/2)]
Ei = [ww/2, (lio/2-ww/2)*np.tan((np.pi-gamma)/2)]
Oa = [-2.5*lio,-2*lio*np.tan((np.pi-gamma)/2)]
Ia = [2.5*lio,-2  *lio*np.tan((np.pi-gamma)/2)]
ax.plot([Oa[0], Eo[0]], [Oa[1], Eo[1]], 'k')
ax.plot([Ia[0], Ei[0]], [Ia[1], Ei[1]], 'k')

ax.set_xlabel('s0 (x coordinate)', fontsize=12)
ax.set_ylabel('l0 (y coordinate)', fontsize=12)
ax.set_title('Optimal Needle Geometry and Desired Points', fontsize=14)
ax.axhline(0, color='black', linewidth=0.8, linestyle='--')  # Reference line
ax.axvline(0, color='black', linewidth=0.8, linestyle='--')
ax.legend()
ax.grid(True)

# Adjust axis limits for better visibility
ax.set_xlim([-1.2*radius, 1.2*radius])
ax.set_ylim([-2 * lio, lio])

# Show the plot
plt.show()
