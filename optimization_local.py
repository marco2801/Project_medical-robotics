import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# Constants and initial parameters
lio = 11  # Length between the desired entry/exit points (mm)
ww = 5  # Wound width, gap between the vessels (to be tuned)(mm)
lins = 10  # Minimum length that can be grasped by the needle (mm)
hti = 8  # Input to stop unwanted needle-tissue contact at the end of the suture (mm)

#optimization output

# s0,l0= coordinates of the needle center 
# s0= coordinate along the vessel axis
# l0= coordinate of the height of the needle center
#dc= needle diameter
#an= needle shapes(array)

gamma = np.pi  # Angle between the two vessels in the suture plane
lambda_weights = [1, 1, 1, 1, 1, 1]  # Optimization weights for suture parameters
an_values = [1 / 4, 3 / 8, 1 / 2, 5 / 8]  # Discrete values for the needle shape

# Limits for normalization
delta_min = [0, 0, 0, 0, 0, 0]  # Minimum values for the parameters of the objective function for the normalization
delta_max = [np.pi / 2, lio / 2, lio, lio / 2, np.pi / 2, lio / 2]  # Maximum values "   "   "   "   "   "   "   "   "   "

# Normalized cost function
def cost_function(needle_vars, *args):
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args

    # Calculate suture parameters
    t = (lio - ww) / (2 * np.cos((np.pi - gamma) / 2)) # distance between the ideal entry/exit point and the edge of vessel
    alpha_1 = np.arcsin(np.clip(2 * np.sin(gamma / 2) /dc * (l0 - np.tan((np.pi - gamma) / 2)) *(lio / 2 + s0), -1, 1))# angle between tissue and the line which links the needle center and the actual exit point
    alpha_2 = np.arcsin(np.clip(2 * np.sin(gamma / 2) / dc * (l0 - np.tan((np.pi - gamma) / 2)) * (lio / 2 - s0), -1, 1))# angle between tissue and the line which links the needle center and the actual entry point
    beta_in = np.pi / 2 + alpha_2 #entry angle
    beta_out = np.pi / 2 + alpha_1 #exit angle
    dh = abs(-dc / 2 + l0 - t * np.sin((np.pi - gamma) / 2))# needle depth
    sn = abs(s0)# needle-wound symmetry
    ein = (-dc / 2 * np.cos(alpha_2 + (np.pi - gamma) / 2) + lio / 2 - s0) / (np.cos((np.pi - gamma) / 2)) # entry error between actual and desired points
    eout = (-dc / 2 * np.cos(alpha_1 + (np.pi - gamma) / 2) + lio / 2 + s0) / (np.cos((np.pi - gamma) / 2)) # exit error between actual and desired points

    # Normalized deviations
    terms = [beta_in - np.pi / 2, ein, dh - lio / 2, sn, beta_out - np.pi / 2, eout] # delta i, in the paper
    normalized_terms = [
        (terms[i] - delta_min[i]) / (delta_max[i] - delta_min[i])
        for i in range(len(terms))
    ]

    # Calculating the normalized cost function
    weighted_terms = [lambda_weights[i] * abs(normalized_terms[i]) for i in range(len(normalized_terms))]
    return sum(weighted_terms)

# Constraints
def bite_time_constraint(needle_vars, *args): # Bite time constrain= rear end of the needle is not in contact with the tissue 
    s0, l0, dc = needle_vars
    gamma, lio, ww, lambda_weights, an, delta_min, delta_max = args
    t = (lio - ww) / (2 * np.cos((np.pi - gamma) / 2))
    alpha_2 = np.arcsin(np.clip(2 * np.sin(gamma / 2) / dc * (l0 - np.tan((np.pi - gamma) / 2)) * (lio / 2 - s0), -1, 1))
    ein = (-dc / 2 * np.cos(alpha_2 + (np.pi - gamma) / 2) + lio / 2 - s0) / (np.cos((np.pi - gamma) / 2))
    qy = np.sin(2 * np.pi * an) * (ww / 2 + (t - ein) * np.cos((np.pi - gamma) / 2) - s0) + np.cos(2 * np.pi * an) * (ein * np.sin((np.pi - gamma) / 2) - l0) + l0 # y coordinate of the rear end of the needle (in plane paper)
    return qy - t * np.sin((np.pi - gamma) / 2) - hti

# Other constraints remain unchanged
# (switching_time_lg_constraint, switching_time_IaOa_constraint, etc.)

# Optimization configuration
bounds = [(-lio / 2, lio / 2),  # s0
          (0, lio),             # l0
          (10, 77)]             # dc (maximum clinical needle diameter)

initial_guess = [0, 0, 30]  # Reasonable initial values to be tuned

# Loop for each discrete value of an
best_solution = None
best_cost = float('inf')

for an in an_values:
    # New switching time constraints
    constraints = [
        {'type': 'ineq', 'fun': bite_time_constraint, 'args': (gamma, lio, ww, lambda_weights, an, delta_min, delta_max)},
        # Add other constraints here
    ]

    # Solve the optimization for this value of an
    result = minimize(
        cost_function,
        initial_guess,
        args=(gamma, lio, ww, lambda_weights, an, delta_min, delta_max),
        bounds=bounds,
        constraints=constraints,
        method='trust-constr',  # Sequential Least Squares Quadratic Programming
        options={'disp': True}
    )

    # Update the best solution
    if result.success and result.fun < best_cost:
        best_cost = result.fun
        best_solution = (result.x, an)

# Display the results
if best_solution:
    optimal_vars, optimal_an = best_solution
    print(f"Optimal solution found: s0={optimal_vars[0]:.2f}, l0={optimal_vars[1]:.2f}, "
          f"dc={optimal_vars[2]:.2f}, an={optimal_an:.2f}")
else:
    print("The optimization did not find a valid solution.")
