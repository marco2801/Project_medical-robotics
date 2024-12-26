import numpy as np
from scipy.optimize import brute
from scipy.optimize import fmin
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# input costants (geoemtric parameters of the suture)
lio = 15 # length between actual and desired entry/exit points (mm) -- 3 times ww in the paper
ww = 5  # wound width, space between vessel (mm)
lins = 10  # minimum grasping needle length (mm)
hti = 8  # Input to stop unwanted needle-tissue contact at the end of the suture (mm)

gamma = np.pi  # angle between vessels
lambda_weights = [1, 1, 1, 1, 1, 1]  # weights for suture parameters
an_values = [1/4, 3/8, 1/2, 5/8]  # discrete values of needle shape

# DELTA_order: beta_in, e_in, dh, sn, beta_out, e_out
delta_min = [0, 0, 0, 0, 0, 0]  # least feasible value for the errors between actual and desirerd results
delta_max = [np.pi / 2, lio / 2, lio, lio / 2, np.pi / 2, lio / 2]  # worst possible errors

# COST FUNCTION
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
    beta_in = np.pi/2 + alpha_2
    beta_out = np.pi/2 + alpha_1
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
    return sum(weighted_terms)

# Constraints

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

# Wrapper per la funzione di costo con vincoli

def cost_function_brute(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max):
    # Verifica dei vincoli
    if not bite_time_constraint(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max) >= 0:
        return np.inf

    # Restituisce la funzione di costo
    return cost_function(needle_vars, gamma, lio, ww, lambda_weights, an, delta_min, delta_max)

# Configurazione del metodo brute
ranges = [
    (-lio / 2, lio / 2),  # s0
    (0, lio),             # l0
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
        finish= minimize,
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
    print(f"Optimal solution found using brute force: s0={optimal_vars[0]:.2f}, "
          f"l0={optimal_vars[1]:.2f}, dc={optimal_vars[2]:.2f}, an={optimal_an:.2f}")

    # Genera un paesaggio della funzione di costo
    s0_vals = np.linspace(-lio / 2, lio / 2, 50)  # Variazioni di s0
    l0_vals = np.linspace(0, lio, 50)            # Variazioni di l0
    costs = np.zeros((len(s0_vals), len(l0_vals)))

    for i, s0 in enumerate(s0_vals):
        for j, l0 in enumerate(l0_vals):
            costs[i, j] = cost_function_brute(
                (s0, l0, optimal_vars[2]),  # Usa il valore ottimale per dc
                gamma, lio, ww, lambda_weights, optimal_an, delta_min, delta_max
            )

    # Visualizza il paesaggio della funzione di costo
    import matplotlib.pyplot as plt

    plt.figure(figsize=(8, 6))
    plt.contourf(s0_vals, l0_vals, costs, levels=50, cmap='viridis')
    plt.plot(optimal_vars[0], optimal_vars[1], 'ro', label='Optimal Solution')
    plt.legend()
    plt.colorbar(label='Cost')
    plt.xlabel('s0')
    plt.ylabel('l0')
    plt.title(f'Cost Landscape (dc={optimal_vars[2]:.2f}, an={optimal_an:.2f})')
    plt.show()
else:
    print("The brute force optimization did not find a valid solution.")

