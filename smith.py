import numpy as np
import control as ctrl
import matplotlib.pyplot as plt
import scipy.io as sio

# Carregar o dataset
file_path = 'Dataset_Grupo1.mat'
data = sio.loadmat(file_path)

# Extraindo tempo, degrau (entrada) e temperatura (saída)
if data['TARGET_DATA____ProjetoC213_Degrau'].shape[0] > data['TARGET_DATA____ProjetoC213_Degrau'].shape[1]:
    tempo = data['TARGET_DATA____ProjetoC213_Degrau'][:, 0]
    degrau = data['TARGET_DATA____ProjetoC213_Degrau'][:, 1]
    temperatura = data['TARGET_DATA____ProjetoC213_PotenciaMotor'][:, 1]
else:
    tempo = data['TARGET_DATA____ProjetoC213_Degrau'][0, :]
    degrau = data['TARGET_DATA____ProjetoC213_Degrau'][1, :]
    temperatura = data['TARGET_DATA____ProjetoC213_PotenciaMotor'][1, :]

# Cálculo do ganho estático (k)
k_smith = temperatura[-1] / degrau[-1]

# Estimativa do atraso de transporte (theta) e constante de tempo (tau)
theta_smith = 0
tau_smith = 0

for i in range(len(temperatura)):
    if temperatura[i] != 0 and theta_smith == 0:
        theta_smith = tempo[i - 1]
    if temperatura[i] >= 0.6321 * temperatura[-1]:
        tau_smith = tempo[i] - theta_smith
        break

# Exibindo os valores de k, tau e theta
print(f"Ganho (k) = {k_smith:.4f}")
print(f"Constante de Tempo (τ) = {tau_smith:.4f}")
print(f"Atraso de Transporte (θ) = {theta_smith:.4f}")

# Função de transferência do sistema sem atraso
sys_smith = ctrl.TransferFunction([k_smith], [tau_smith, 1])

# Aproximação de Padé para o atraso de transporte
num_pade, den_pade = ctrl.pade(theta_smith, 1)  # Ordem 1 da aproximação de Padé
sys_smith_delay = ctrl.TransferFunction(num_pade, den_pade)

# Combinar a função de transferência com o atraso
sys_with_delay_smith = ctrl.series(sys_smith_delay, sys_smith)

# -------------------------
# Sintonia do Controlador PID (Ziegler-Nichols - Malha Aberta)
# -------------------------
Kp_zn = 1.2 * tau_smith / (k_smith * theta_smith)
Ti_zn = 2 * theta_smith
Td_zn = 0.5 * theta_smith

pid_zn = ctrl.TransferFunction([Kp_zn * Td_zn, Kp_zn, Kp_zn / Ti_zn], [1, 0])
sys_closed_zn = ctrl.feedback(ctrl.series(pid_zn, sys_with_delay_smith))

# -------------------------
# Sintonia do Controlador PID (CHR com Sobrevalor)
# -------------------------
Kp_chr = 0.95 * tau_smith / (k_smith * theta_smith)
Ti_chr = 1.357 * tau_smith
Td_chr = 0.473 * theta_smith

pid_chr = ctrl.TransferFunction([Kp_chr * Td_chr, Kp_chr, Kp_chr / Ti_chr], [1, 0])
sys_closed_chr = ctrl.feedback(ctrl.series(pid_chr, sys_with_delay_smith))

# -------------------------
# Sintonia IMC (Internal Model Control)
# -------------------------
lambda_imc = 0.5 * tau_smith  # Escolha de λ como metade da constante de tempo para bom desempenho

# Controlador PID para IMC
Kp_imc = tau_smith / (k_smith * (lambda_imc + theta_smith))
Ti_imc = tau_smith
Td_imc = (lambda_imc * theta_smith) / (lambda_imc + theta_smith)

pid_imc = ctrl.TransferFunction([Kp_imc * Td_imc, Kp_imc, Kp_imc / Ti_imc], [1, 0])
sys_closed_imc = ctrl.feedback(ctrl.series(pid_imc, sys_with_delay_smith))

# -------------------------
# Resposta ao Degrau (Sistemas Controlados)
# -------------------------
# Malha Fechada com Ziegler-Nichols
t_zn, y_zn = ctrl.step_response(sys_closed_zn, T=tempo)

# Malha Fechada com CHR com Sobrevalor
t_chr, y_chr = ctrl.step_response(sys_closed_chr, T=tempo)

# Malha Fechada com IMC
t_imc, y_imc = ctrl.step_response(sys_closed_imc, T=tempo)

# Plotar as respostas
plt.figure()
plt.plot(t_zn, y_zn, label="Ziegler-Nichols (Malha Aberta)")
plt.plot(t_chr, y_chr, label="CHR com Sobrevalor")
plt.plot(t_imc, y_imc, label="IMC (λ = 0.5 τ)")
plt.plot(tempo, temperatura, 'r--', label="Dados Reais")
plt.xlabel('Tempo [s]')
plt.ylabel('Temperatura [°C]')
plt.title('Comparação de Sintonias: Ziegler-Nichols, CHR e IMC')
plt.legend(loc='lower right')
plt.grid(True)
plt.show()

# -------------------------
# Comentando o valor de λ (IMC)
# -------------------------
print(f"Valor de λ escolhido para IMC = {lambda_imc:.4f}")
print("Escolhemos λ como metade da constante de tempo do sistema (τ/2) para um equilíbrio entre rapidez e robustez.")
