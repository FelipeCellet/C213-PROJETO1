import numpy as np
import control as ctrl
import matplotlib.pyplot as plt
import scipy.io as sio
import tkinter as tk
from tkinter import ttk

# Função para calcular o desempenho (tempo de subida, acomodação, pico)
def calc_performance(t, y, target):
    rise_time_idx = np.where((y >= 0.1 * target) & (y <= 0.9 * target))[0]
    rise_time = t[rise_time_idx[-1]] - t[rise_time_idx[0]] if len(rise_time_idx) > 0 else np.nan

    settle_time_idx = np.where(np.abs(y - target) <= 0.02 * target)[0]
    settle_time = t[settle_time_idx[-1]] - t[settle_time_idx[0]] if len(settle_time_idx) > 0 else np.nan

    steady_state_error = np.abs(target - y[-1])
    peak_value = np.max(y)

    return rise_time, settle_time, steady_state_error, peak_value

# Função para processar os dados de acordo com o método e sintonia PID
def process_data(method, pid_tuning):
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
    k = temperatura[-1] / degrau[-1]

    # Estimativa do atraso de transporte (theta) e constante de tempo (tau) de acordo com o método
    theta = None
    tau = None

    if method == 'Smith':
        # Método Smith para calcular tau e theta
        y1 = 0.283 * temperatura[-1]
        y2 = 0.632 * temperatura[-1]

        t1_idx = np.where(temperatura >= y1)[0][0]
        t2_idx = np.where(temperatura >= y2)[0][0]

        t1 = tempo[t1_idx]
        t2 = tempo[t2_idx]

        tau = 1.5 * (t2 - t1)
        theta = t2 - tau

    elif method == 'Sundaresan':
        if len(np.diff(temperatura)) > 0:
            theta = tempo[np.argmax(np.diff(temperatura))]
            tau = (tempo[np.argmax(temperatura)] - theta) / 2
        else:
            print("Erro: Não foi possível calcular tau e theta para o método Sundaresan.")
            return

    if tau is None or theta is None:
        print(f"Erro: Não foi possível calcular tau e theta para o método {method}.")
        return

    # Função de transferência do sistema sem atraso
    sys = ctrl.TransferFunction([k], [tau, 1])

    # Aproximação de Padé para o atraso de transporte
    num_pade, den_pade = ctrl.pade(theta, 1)
    sys_delay = ctrl.TransferFunction(num_pade, den_pade)

    # Combinar a função de transferência com o atraso
    sys_with_delay = ctrl.series(sys_delay, sys)

    # Sintonia do Controlador PID
    if pid_tuning == 'Ziegler-Nichols':
        Kp = 1.2 * tau / (k * theta)
        Ti = 2 * theta
        Td = 0.5 * theta

        pid = ctrl.TransferFunction([Kp * Td, Kp, Kp / Ti], [1, 0])
        sys_closed = ctrl.feedback(ctrl.series(pid, sys_with_delay))
        pid_label = 'Ziegler-Nichols'

    elif pid_tuning == 'CHR':
        Kp = 0.95 * tau / (k * theta)
        Ti = 1.357 * tau
        Td = 0.473 * theta

        pid = ctrl.TransferFunction([Kp * Td, Kp, Kp / Ti], [1, 0])
        sys_closed = ctrl.feedback(ctrl.series(pid, sys_with_delay))
        pid_label = 'CHR'

    # Resposta do Sistema em Malha Aberta
    t_open, y_open = ctrl.step_response(sys_with_delay, T=tempo)

    # Resposta do Sistema em Malha Fechada
    t_closed, y_closed = ctrl.step_response(sys_closed, T=tempo)

    # Calcular desempenho
    target_value = degrau[-1]
    rise_time_open, settle_time_open, steady_state_error_open, peak_open = calc_performance(t_open, y_open, target_value)
    rise_time_closed, settle_time_closed, steady_state_error_closed, peak_closed = calc_performance(t_closed, y_closed, target_value)

    # Exibir os resultados no terminal
    results_str = (
        f"Resultados para o método {method} com sintonia {pid_label}:\n\n"
        f"Malha Aberta:\n\n"
        f"Tempo de subida = {rise_time_open:.2f} s,\n"
        f"Tempo de acomodação = {settle_time_open:.2f} s,\n"
        f"Pico = {peak_open:.4f}\n\n"
        f"Malha Fechada ({pid_label}):\n\n"
        f"Tempo de subida = {rise_time_closed:.2f} s,\n"
        f"Tempo de acomodação = {settle_time_closed:.2f} s,\n"
        f"Pico = {peak_closed:.4f}\n\n"
        "\nDiferenças observadas:\n"
        "- A malha aberta tem um tempo de subida e de acomodação maior, além de um erro estacionário significativo.\n"
    )
    if pid_label == 'Ziegler-Nichols':
        results_str += f"- A malha fechada ({pid_label}) apresentou um tempo de subida mais rápido, mas com possível overshoot.\n"
    else:
        results_str += f"- A malha fechada ({pid_label}) mostrou um desempenho mais equilibrado, com menor sobrevalor e bom tempo de acomodação.\n"

    print(results_str)  # Exibir no terminal

    # Plotar os gráficos
    plt.figure()
    plt.plot(t_open, y_open, label="Malha Aberta")
    plt.plot(t_closed, y_closed, label=f"Malha Fechada - {pid_label}")
    plt.xlabel('Tempo [s]')
    plt.ylabel('Temperatura [°C]')
    plt.title(f'Comparação: Malha Aberta vs Malha Fechada ({pid_label}) usando {method}')
    plt.legend(loc='lower right')
    plt.grid(True)

    # Adicionar as informações de desempenho e os resultados diretamente no gráfico
    props = dict(boxstyle='round', facecolor='white', alpha=0.6)
    plt.text(tempo[-1] * 0.5, max(temperatura) * 0.5, results_str, fontsize=8, bbox=props)

    plt.show()

# Função para comparar Ziegler-Nichols e CHR
def compare_pids(method):
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
    k = temperatura[-1] / degrau[-1]

    # Estimativa do atraso de transporte (theta) e constante de tempo (tau)
    theta = None
    tau = None

    if method == 'Smith':
        y1 = 0.283 * temperatura[-1]
        y2 = 0.632 * temperatura[-1]

        t1_idx = np.where(temperatura >= y1)[0][0]
        t2_idx = np.where(temperatura >= y2)[0][0]

        t1 = tempo[t1_idx]
        t2 = tempo[t2_idx]

        tau = 1.5 * (t2 - t1)
        theta = t2 - tau

    elif method == 'Sundaresan':
        if len(np.diff(temperatura)) > 0:
            theta = tempo[np.argmax(np.diff(temperatura))]
            tau = (tempo[np.argmax(temperatura)] - theta) / 2
        else:
            print("Erro: Não foi possível calcular tau e theta para o método Sundaresan.")
            return

    if tau is None or theta is None:
        print(f"Erro: Não foi possível calcular tau e theta para o método {method}.")
        return

    # Função de transferência do sistema sem atraso
    sys = ctrl.TransferFunction([k], [tau, 1])

    # Aproximação de Padé para o atraso de transporte
    num_pade, den_pade = ctrl.pade(theta, 1)
    sys_delay = ctrl.TransferFunction(num_pade, den_pade)

    # Combinar a função de transferência com o atraso
    sys_with_delay = ctrl.series(sys_delay, sys)

    # Ziegler-Nichols
    Kp_zn = 1.2 * tau / (k * theta)
    Ti_zn = 2 * theta
    Td_zn = 0.5 * theta
    pid_zn = ctrl.TransferFunction([Kp_zn * Td_zn, Kp_zn, Kp_zn / Ti_zn], [1, 0])
    sys_closed_zn = ctrl.feedback(ctrl.series(pid_zn, sys_with_delay))

    # CHR
    Kp_chr = 0.95 * tau / (k * theta)
    Ti_chr = 1.357 * tau
    Td_chr = 0.473 * theta
    pid_chr = ctrl.TransferFunction([Kp_chr * Td_chr, Kp_chr, Kp_chr / Ti_chr], [1, 0])
    sys_closed_chr = ctrl.feedback(ctrl.series(pid_chr, sys_with_delay))

    # Resposta do Sistema com Ziegler-Nichols
    t_closed_zn, y_closed_zn = ctrl.step_response(sys_closed_zn, T=tempo)

    # Resposta do Sistema com CHR
    t_closed_chr, y_closed_chr = ctrl.step_response(sys_closed_chr, T=tempo)

    # Calcular desempenho para Ziegler-Nichols
    target_value = degrau[-1]
    rise_time_zn, settle_time_zn, steady_state_error_zn, peak_zn = calc_performance(t_closed_zn, y_closed_zn, target_value)

    # Calcular desempenho para CHR
    rise_time_chr, settle_time_chr, steady_state_error_chr, peak_chr = calc_performance(t_closed_chr, y_closed_chr, target_value)

    # Imprimir os resultados no terminal
    print(f"\nResultados para o método {method}:")
    print(f"Ziegler-Nichols: Tempo de subida = {rise_time_zn:.2f} s, Tempo de acomodação = {settle_time_zn:.2f} s, Pico = {peak_zn:.4f}")
    print(f"CHR: Tempo de subida = {rise_time_chr:.2f} s, Tempo de acomodação = {settle_time_chr:.2f} s, Pico = {peak_chr:.4f}\n")

    # Explicação sobre as diferenças com valores comparativos
    print("Explicação sobre as diferenças:")
    print(f"- Ziegler-Nichols oferece uma resposta mais rápida com tempo de subida menor ({rise_time_zn:.2f} s), mas com overshoot maior (pico = {peak_zn:.4f}).")
    print(f"- CHR busca uma resposta mais equilibrada com tempo de acomodação menor ({settle_time_chr:.2f} s) e menor overshoot (pico = {peak_chr:.4f}).\n")

    # Plotar os gráficos comparativos
    plt.figure()
    plt.plot(t_closed_zn, y_closed_zn, label="Ziegler-Nichols")
    plt.plot(t_closed_chr, y_closed_chr, label="CHR")
    plt.xlabel('Tempo [s]')
    plt.ylabel('Temperatura [°C]')
    plt.title(f'Comparação: Ziegler-Nichols vs CHR ({method})')
    plt.legend(loc='lower right')
    plt.grid(True)
    plt.show()

# Função para a interface gráfica
def create_gui():
    root = tk.Tk()
    root.title('Seleção de Método de Modelo e Sintonia PID')

    label_method = tk.Label(root, text="Selecione o método de modelo:")
    label_method.pack(pady=10)

    selected_method = tk.StringVar()

    def on_method_select(method):
        selected_method.set(method)
        pid_gui(method)

    def pid_gui(method):
        pid_window = tk.Toplevel(root)
        pid_window.title("Seleção de Sintonia PID")

        label_pid = tk.Label(pid_window, text=f"Método {method} selecionado. Agora, escolha a sintonia PID:")
        label_pid.pack(pady=10)

        def on_pid_select(pid_method):
            process_data(method, pid_method)

        button_zn = ttk.Button(pid_window, text="Ziegler-Nichols", command=lambda: on_pid_select('Ziegler-Nichols'))
        button_zn.pack(pady=5)

        button_chr = ttk.Button(pid_window, text="CHR", command=lambda: on_pid_select('CHR'))
        button_chr.pack(pady=5)

    def on_compare_select():
        compare_pids(selected_method.get())

    button_smith = ttk.Button(root, text="Smith", command=lambda: on_method_select('Smith'))
    button_smith.pack(pady=5)

    button_sundaresan = ttk.Button(root, text="Sundaresan", command=lambda: on_method_select('Sundaresan'))
    button_sundaresan.pack(pady=5)

    button_compare = ttk.Button(root, text="Comparar Ziegler-Nichols e CHR", command=on_compare_select)
    button_compare.pack(pady=10)

    root.mainloop()

# Executar a interface gráfica
create_gui()

