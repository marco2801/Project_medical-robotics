import numpy as np
from scipy.optimize import brute
from scipy.optimize import fmin
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# input costants (geoemtric parameters of the suture)
lio = 15 # length between actual and desired entry/exit points (mm) -- 3 times ww in the paper
ww = 5  # wound width, space between vessel (mm)
lins = 10  # minimum grasping needle length (mm)
hti = 8  # Input to stop unwanted needle-tissue contact at the end of the suture (mm)

# Tunable parameters
gamma = np.pi # angle between vessels
lambda_weights = [1, 1, 1, 1, 1, 1]  # weights for suture parameters
an_values = [1/4, 3/8, 1/2, 5/8]  # discrete values of needle shape

# DELTA_order: beta_in, e_in, dh, sn, beta_out, e_out (IS IT CORRECT?)
delta_min = [0, 0, 0, 0, 0, 0]  # least feasible value for the errors between actual and desirerd results
delta_max = [np.pi/2, lio / 2, lio, lio / 2, np.pi/2, lio / 2]  # worst possible errors

# COST FUNCTION (without constraints)
def cost_function(needle_vars, *args):
    s0, l0, dc = needle_vars            
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    # Suture parameters computation
    # t = distance between desired entry (Id) and input edge of wound (Ei)
    t = (lio - ww) / (2 * np.cos((np.pi - gamma) / 2))
    # output angle between tissue surface and the needle center
    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * (l0 - np.tan((np.pi - gamma)/2)) * (lio/2 + s0),
        -1, 1
    ))
    # input angle between tissue surface and the needle center
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * (l0 - np.tan((np.pi - gamma)/2)) * (lio/2 - s0),
        -1, 1
    ))

    # SUTURE PARAMETERS subjected to lambda weights
    # entry and exit angles [rad]
    beta_in = np.clip(np.pi/2 + alpha_2,0, np.pi)
    beta_out = np.clip(np.pi/2 + alpha_1,0, np.pi)
    #suture depth [mm]
    dh = abs(-dc/2 + l0 - t * np.sin((np.pi-gamma)/2))
    # simmetry [mm]
    sn = abs(s0)
    # entry and exit distances between desired and actual points [mm]
    ein = (-dc / 2 * np.cos(alpha_2 + (np.pi - gamma) / 2) + lio / 2 - s0) / (np.cos((np.pi - gamma) / 2))
    eout = (-dc / 2 * np.cos(alpha_1 + (np.pi - gamma) / 2) + lio / 2 + s0) / (np.cos((np.pi - gamma) / 2))

    # Normalized deviations
    terms = [beta_in - np.pi / 2, ein, dh - lio / 2, sn, beta_out - np.pi / 2, eout]

    normalized_terms = [
        (terms[i] - delta_min[i]) / (delta_max[i] - delta_min[i])
        for i in range(len(terms))
    ]

    # Normalized cost function
    weighted_terms = [
        lambda_weights[i] * abs(normalized_terms[i]) for i in range(len(normalized_terms))
    ]
    return sum(weighted_terms)/sum(lambda_weights)

## CONSTRAINTS DEFINITION

#1) BT = BITE TIME
def bite_time_constraint(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args
    t = (lio - ww) / (2 * np.cos((np.pi - gamma) / 2))
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * (l0 - np.tan((np.pi - gamma) / 2)) * (lio / 2 - s0),
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
        2 * np.sin(gamma/2) / dc * (l0 - np.tan((np.pi - gamma)/2)) * (lio/2 + s0),
        -1, 1
    ))
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * (l0 - np.tan((np.pi - gamma) / 2)) * (lio / 2 - s0),
        -1, 1
    ))
    lg = (np.pi*an*dc - dc/2*(gamma-alpha_1-alpha_2))/2

    return lg-lins

#2.2) SW = SWITCHING TIME constraint --> the needle passes externally to the wound
def switching_time_constraint_2(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma/2) / dc * (l0 - np.tan((np.pi - gamma)/2)) * (lio/2 + s0),
        -1, 1
    ))
    alpha_2 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * (l0 - np.tan((np.pi - gamma) / 2)) * (lio / 2 - s0),
        -1, 1
    ))
    Ia_Oa = dc*np.sin((gamma-alpha_1-alpha_2)/2)

    return Ia_Oa-ww

#2.3) SW = SWITCHING TIME constraint --> the needle must be inside the tissue
def switching_time_constraint_3(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    t = (lio - ww) / (2 * np.cos((np.pi - gamma) / 2))
    dh_coord = -dc/2 +l0 -t*np.sin((np.pi-gamma)/2)

    return dh_coord

#3) ET = EXTRACTION TIME
def extraction_time_constraint(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    t = (lio - ww) / (2 * np.cos((np.pi - gamma) / 2))
    alpha_1 = np.arcsin(np.clip(
        2 * np.sin(gamma / 2) / dc * (l0 - np.tan((np.pi - gamma) / 2)) * (lio / 2 + s0),
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
    
    # constraints verification
    if not bite_time_constraint(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf
    if not switching_time_constraint_1(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf
    if not switching_time_constraint_2(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf
    if not switching_time_constraint_3(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) <= 0:
        return np.inf
    if not extraction_time_constraint(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf

    return cost_function(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max)

# Feasible ranges
ranges = [
    (-lio / 2, lio / 2),  # s0
    (0, 3*lio),           # l0
    (10, 77)              # dc
]

Ns = 50  # Risoluzione della griglia

# Ciclo sui valori discreti di an
best_solution = None
best_cost = float('inf')

for an in an_values:
    # Ottimizzazione brute
    result = brute(
        cost_function_brute,
        ranges=ranges,
        args=(gamma, lio, ww, lambda_weights, an, delta_min, delta_max),
        full_output=True,
        finish= fmin,
        Ns=Ns,
        disp = True
    )

    # Estrai il minimo trovato
    min_cost, min_vars = result[1], result[0]

    # Aggiorna la soluzione migliore
    if min_cost < best_cost:
        best_cost = min_cost
        best_solution = (min_vars, an)

# Dopo aver trovato la soluzione ottimale
# Dopo aver trovato la soluzione ottimale
if best_solution:
    optimal_vars, optimal_an = best_solution
    print(f"Optimal solution found using brute force: Cost function value C = {best_cost}, s0={optimal_vars[0]:.2f}, "
          f"l0={optimal_vars[1]:.2f}, dc={optimal_vars[2]:.2f}, an={optimal_an:.2f}")

    # Configura i valori delle griglie
    s0_vals = np.linspace(-lio / 2, lio / 2, 30)  
    l0_vals = np.linspace(0, 2*lio, 30)            
    dc_vals = np.linspace(10, 77, 30)           

    # Creazione figure
    fig = plt.figure(figsize=(16, 12))

    # Genera 4 grafici, uno per ciascun valore di an
    for idx, an in enumerate(an_values):

        # Griglia 3D per s0, l0, dc
        S0, L0, DC = np.meshgrid(s0_vals, l0_vals, dc_vals)
        
        # Crea il sottografico
        ax = fig.add_subplot(2, 2, idx+1, projection='3d')
        ax.set_title(f"Cost Landscape for an={an:.2f}")
        ax.set_xlabel('s0')
        ax.set_ylabel('l0')
        ax.set_zlabel('dc')
        
        # Calcolo dei costi sulla griglia
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
       
        # Scatter dei dati con color mapping
        scatter = ax.scatter(S0.flatten(), L0.flatten(), DC.flatten(), c=Costs.flatten(), cmap='viridis')
        fig.colorbar(scatter, ax=ax, shrink=0.5, aspect=10, label="Cost")
    
    # Mostra il grafico
    plt.tight_layout()
    plt.show()

else:
    print("The brute force optimization did not find a valid solution.")

