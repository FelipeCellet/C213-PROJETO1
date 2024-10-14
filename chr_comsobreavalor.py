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
k_chr = np.max(saida)  # Ganho estático
tau_chr = 2.0  # Constante de tempo estimada
theta_chr = 0.5  # Atraso de transporte estimado

# Função de transferência do sistema (CHR com Sobrevalor)
num_chr = [k_chr]
den_chr = [tau_chr, 1]
sys_chr = ctrl.TransferFunction(num_chr, den_chr)

# Aproximação de Padé para o atraso de transporte
sys_delay_chr = ctrl.pade(theta_chr, 1)
sys_with_delay_chr = ctrl.series(ctrl.TransferFunction(*sys_delay_chr), sys_chr)

# Resposta em malha aberta
t_chr, y_chr = ctrl.step_response(sys_with_delay_chr, T=tempo)

# Calcular o Erro Quadrático Médio (EQM)
eqm_chr = mean_squared_error(saida, np.interp(tempo, t_chr, y_chr))

# Plotar resposta em malha aberta
plt.plot(tempo, saida, label="Saída Real")
plt.plot(t_chr, y_chr, label=f"Modelo CHR - Malha Aberta (EQM={eqm_chr:.4f})")
plt.title(f'Método CHR com Sobrevalor - Malha Aberta\nk={k_chr}, τ={tau_chr}, θ={theta_chr}')
plt.xlabel('Tempo [s]')
plt.ylabel('Saída')
plt.legend()
plt.grid(True)
plt.show()

# Sintonia PID - Usando CHR com sobrevalor
Kp_chr = 0.95 * tau_chr / (k_chr * theta_chr)
Ti_chr = 1.357 * tau_chr
Td_chr = 0.473 * theta_chr
pid_chr = ctrl.TransferFunction([Kp_chr * Td_chr, Kp_chr, Kp_chr / Ti_chr], [1, 0])

# Malha fechada
sys_closed_loop_chr = ctrl.feedback(ctrl.series(pid_chr, sys_with_delay_chr))

# Resposta em malha fechada
t_chr_closed, y_chr_closed = ctrl.step_response(sys_closed_loop_chr, T=tempo)

# Calcular o EQM em malha fechada
eqm_chr_closed = mean_squared_error(saida, np.interp(tempo, t_chr_closed, y_chr_closed))

# Plotar resposta em malha fechada
plt.plot(tempo, saida, label="Saída Real")
plt.plot(t_chr_closed, y_chr_closed, label=f"Modelo CHR - Malha Fechada (EQM={eqm_chr_closed:.4f})")
plt.title(f'Método CHR com Sobrevalor - Malha Fechada com Controlador PID\nk={k_chr}, τ={tau_chr}, θ={theta_chr}')
plt.xlabel('Tempo [s]')
plt.ylabel('Saída')
plt.legend()
plt.grid(True)
plt.show()