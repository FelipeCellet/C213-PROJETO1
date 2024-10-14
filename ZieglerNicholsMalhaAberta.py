import numpy as np
import control as ctrl
import matplotlib.pyplot as plt
import scipy.io as sio
from sklearn.metrics import mean_squared_error

# Carregar o dataset
file_path = 'Dataset_Grupo1.mat'
data = sio.loadmat(file_path)

# Extraindo entrada, saída e tempo
entrada = data['TARGET_DATA____ProjetoC213_Degrau'][:, 1]  
saida = data['TARGET_DATA____ProjetoC213_PotenciaMotor'][:, 1] 
tempo = data['TARGET_DATA____ProjetoC213_Degrau'][:, 0]  

# Estimativas dos parâmetros baseadas no comportamento do degrau
k_zn = np.max(saida)  # Ganho estático
tau_zn = 2.0  # Constante de tempo estimada
theta_zn = 0.5  # Atraso de transporte estimado

# Função de transferência do sistema (Ziegler-Nichols em Malha Aberta)
num_zn = [k_zn]
den_zn = [tau_zn, 1]
sys_zn = ctrl.TransferFunction(num_zn, den_zn)

# Aproximação de Padé para o atraso de transporte
sys_delay_zn = ctrl.pade(theta_zn, 1)
sys_with_delay_zn = ctrl.series(ctrl.TransferFunction(*sys_delay_zn), sys_zn)

# Resposta em malha aberta
t_zn, y_zn = ctrl.step_response(sys_with_delay_zn, T=tempo)

# Calcular o Erro Quadrático Médio (EQM)
eqm_zn = mean_squared_error(saida, np.interp(tempo, t_zn, y_zn))

# Plotar resposta em malha aberta
plt.plot(tempo, saida, label="Saída Real")
plt.plot(t_zn, y_zn, label=f"Modelo Ziegler-Nichols - Malha Aberta (EQM={eqm_zn:.4f})")
plt.title(f'Método Ziegler-Nichols - Malha Aberta\nk={k_zn}, τ={tau_zn}, θ={theta_zn}')
plt.xlabel('Tempo [s]')
plt.ylabel('Saída')
plt.legend()
plt.grid(True)
plt.show()

# Sintonia PID - Usando Ziegler-Nichols
Kp_zn = 1.2 * tau_zn / (k_zn * theta_zn)
Ti_zn = 2 * theta_zn
Td_zn = theta_zn / 2
pid_zn = ctrl.TransferFunction([Kp_zn * Td_zn, Kp_zn, Kp_zn / Ti_zn], [1, 0])

# Malha fechada
sys_closed_loop_zn = ctrl.feedback(ctrl.series(pid_zn, sys_with_delay_zn))

# Resposta em malha fechada
t_zn_closed, y_zn_closed = ctrl.step_response(sys_closed_loop_zn, T=tempo)

# Calcular o EQM em malha fechada
eqm_zn_closed = mean_squared_error(saida, np.interp(tempo, t_zn_closed, y_zn_closed))

# Plotar resposta em malha fechada
plt.plot(tempo, saida, label="Saída Real")
plt.plot(t_zn_closed, y_zn_closed, label=f"Modelo Ziegler-Nichols - Malha Fechada (EQM={eqm_zn_closed:.4f})")
plt.title(f'Método Ziegler-Nichols - Malha Fechada com Controlador PID\nk={k_zn}, τ={tau_zn}, θ={theta_zn}')
plt.xlabel('Tempo [s]')
plt.ylabel('Saída')
plt.legend()
plt.grid(True)
plt.show()