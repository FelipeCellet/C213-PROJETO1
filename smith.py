import numpy as np
import control as ctrl
import matplotlib.pyplot as plt
import scipy.io as sio

# Carregar o dataset
file_path = 'Dataset_Grupo1.mat'
data = sio.loadmat(file_path)

# Mensagem explicando o uso do método Smith
print("O método de Smith foi escolhido porque é ideal para sistemas de primeira ordem com atraso de transporte.")
print(
    "Ele permite estimar com precisão o atraso e ajustar o modelo do sistema, especialmente quando os dados experimentais têm pouca incerteza.")

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
# Resposta do Sistema em Malha Aberta
# -------------------------
t_open, y_open = ctrl.step_response(sys_with_delay_smith, T=tempo)

plt.figure()
plt.plot(t_open, y_open, label="Malha Aberta")
plt.plot(tempo, temperatura, 'r--', label="Dados Reais")
plt.xlabel('Tempo [s]')
plt.ylabel('Temperatura [°C]')
plt.title('Resposta do Sistema em Malha Aberta')
plt.legend(loc='lower right')
plt.grid(True)
plt.show()

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
# Resposta ao Degrau (Malha Fechada com Ziegler-Nichols e CHR)
# -------------------------
t_closed_zn, y_closed_zn = ctrl.step_response(sys_closed_zn, T=tempo)
t_closed_chr, y_closed_chr = ctrl.step_response(sys_closed_chr, T=tempo)

# -------------------------
# Plotar as Respostas (Malha Fechada e Malha Aberta)
# -------------------------
plt.figure()
plt.plot(t_open, y_open, label="Malha Aberta")
plt.plot(t_closed_zn, y_closed_zn, label="Malha Fechada - Ziegler-Nichols")
plt.plot(t_closed_chr, y_closed_chr, label="Malha Fechada - CHR")
plt.xlabel('Tempo [s]')
plt.ylabel('Temperatura [°C]')
plt.title('Comparação: Malha Aberta vs Malha Fechada (Ziegler-Nichols e CHR)')
plt.legend(loc='lower right')
plt.grid(True)
plt.show()


# -------------------------
# Cálculo do Desempenho (Tempo de Subida, Acomodação, Erro)
# -------------------------
def calc_performance(t, y, target):
    # Verificar se o sistema atinge 10% e 90% do valor alvo
    if np.max(y) < 0.9 * target or np.min(y) > 0.1 * target:
        print("O sistema não atinge os limites de 10% a 90% do valor final.")
        return np.nan, np.nan, np.abs(target - y[-1])

    # Tempo de subida (de 10% a 90%)
    rise_time_idx = np.where((y >= 0.1 * target) & (y <= 0.9 * target))[0]
    rise_time = t[rise_time_idx[-1]] - t[rise_time_idx[0]] if len(rise_time_idx) > 0 else np.nan

    # Tempo de acomodação (dentro de 2% do valor final)
    settle_time_idx = np.where(np.abs(y - target) <= 0.02 * target)[0]
    settle_time = t[settle_time_idx[-1]] - t[settle_time_idx[0]] if len(settle_time_idx) > 0 else np.nan

    # Erro do processo (erro de estado estacionário)
    steady_state_error = np.abs(target - y[-1])

    return rise_time, settle_time, steady_state_error


# Alvo final (degrau)
target_value = degrau[-1]

# Desempenho da Malha Aberta
rise_time_open, settle_time_open, steady_state_error_open = calc_performance(t_open, y_open, target_value)

# Desempenho da Malha Fechada (Ziegler-Nichols)
rise_time_zn, settle_time_zn, steady_state_error_zn = calc_performance(t_closed_zn, y_closed_zn, target_value)

# Desempenho da Malha Fechada (CHR)
rise_time_chr, settle_time_chr, steady_state_error_chr = calc_performance(t_closed_chr, y_closed_chr, target_value)

# -------------------------
# Exibir os Resultados
# -------------------------
print(
    f"Malha Aberta: Tempo de subida = {rise_time_open:.2f} s, Tempo de acomodação = {settle_time_open:.2f} s, Erro estacionário = {steady_state_error_open:.4f}")
print(
    f"Malha Fechada (Ziegler-Nichols): Tempo de subida = {rise_time_zn:.2f} s, Tempo de acomodação = {settle_time_zn:.2f} s, Erro estacionário = {steady_state_error_zn:.4f}")
print(
    f"Malha Fechada (CHR): Tempo de subida = {rise_time_chr:.2f} s, Tempo de acomodação = {settle_time_chr:.2f} s, Erro estacionário = {steady_state_error_chr:.4f}")

# -------------------------
# Comparação das Diferenças
# -------------------------
print("\nDiferenças observadas:")
print("- A malha aberta tem um tempo de subida e de acomodação maior, além de um erro estacionário significativo.")
print("- A malha fechada (Ziegler-Nichols) apresentou um tempo de subida mais rápido, mas com possível overshoot.")
print("- A malha fechada (CHR) mostrou um desempenho mais equilibrado, com menor sobrevalor e bom tempo de acomodação.")
