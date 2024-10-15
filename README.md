# C213-PROJETO1
 Primerio projeto C213


## Método escolhido: Smith

O método de **Smith** é preferido em situações onde o sistema apresenta uma resposta bem definida e o atraso de transporte pode ser estimado com precisão. Ele é particularmente adequado para sistemas de primeira ordem, especialmente quando os dados experimentais apresentam pouca incerteza.

### Determinação dos Parâmetros
Através do método de Smith, os valores de **ganho (k)**, **constante de tempo (τ)** e **atraso de transporte (θ)** são determinados. Esses valores fornecem uma boa estimativa da função de transferência do modelo.

### Justificativa da Escolha
- **Simplicidade**: O método de Smith é direto e fornece uma boa aproximação para sistemas de primeira ordem.
- **Precisão**: Ele é robusto para estimativas de \(k\), \(\tau\) e \(\theta\), especialmente quando o atraso de transporte é bem observado.
- **Adaptabilidade**: A combinação com a aproximação de Padé lida bem com o atraso de transporte.

## Análise do Sistema em Malha Aberta e Fechada

### Definições Importantes
- **Tempo de subida**: Tempo necessário para o sistema ir de 10% a 90% do valor final.
- **Tempo de acomodação**: Tempo necessário para o sistema estabilizar dentro de uma faixa de ±2% do valor final.
- **Erro do processo**: Diferença entre o valor final esperado e o valor final atingido em regime permanente.

### Comportamento do Sistema
- **Malha Aberta**:
  - **Tempo de subida** maior, pois não há controle ativo.
  - **Tempo de acomodação** pode ser inexistente, com possíveis oscilações.
  - **Erro estacionário** tende a ser maior, sem feedback para correção.
  
- **Malha Fechada**:
  - **Tempo de subida** menor, graças ao ajuste do controlador PID.
  - **Tempo de acomodação** reduzido, já que o controlador estabiliza o sistema mais rápido.
  - **Erro estacionário** reduzido ou nulo, devido ao feedback ativo.

## Sintonização do Controlador PID

A sintonização foi feita usando os métodos de **Ziegler-Nichols (Malha Aberta)** e **CHR com sobrevalor**, além da **Sintonia IMC** para controle baseado no modelo interno.

### Métodos Utilizados

1. **Ziegler-Nichols (Malha Aberta)**:
   - Método tradicional que ajusta \(K_p\), \(T_i\), e \(T_d\) para obter uma resposta relativamente rápida, porém com maior overshoot.
   - Fórmulas:
     - \(K_p = 1.2 \cdot \frac{\tau}{k \cdot \theta}\)
     - \(T_i = 2 \cdot \theta\)
     - \(T_d = 0.5 \cdot \theta\)

2. **CHR com Sobrevalor**:
   - Ajusta os parâmetros PID para reduzir o overshoot e manter um bom tempo de acomodação.
   - Fórmulas:
     - \(K_p = 0.95 \cdot \frac{\tau}{k \cdot \theta}\)
     - \(T_i = 1.357 \cdot \tau\)
     - \(T_d = 0.473 \cdot \theta\)

3. **Sintonia IMC (Internal Model Control)**:
   - Ajusta o controlador com base no parâmetro \(\lambda\), que controla o equilíbrio entre rapidez e robustez.
   - Valor escolhido: \(\lambda = 0.5 \cdot \tau\), para um bom compromisso entre rapidez e robustez.

### Justificativa para o valor de \(\lambda\):
- Um valor de \(\lambda = 0.5 \cdot \tau\) foi escolhido para garantir que o sistema mantenha um equilíbrio adequado entre rapidez de resposta e robustez. Valores muito pequenos de \(\lambda\) poderiam causar uma resposta rápida, mas sensível a ruídos, enquanto valores maiores tornariam o sistema mais robusto, porém mais lento.

## Resultados Esperados
As respostas dos sistemas controlados por **Ziegler-Nichols**, **CHR com sobrevalor** e **IMC** serão plotadas para comparação. O valor de \(\lambda\) escolhido para a sintonia IMC será exibido com a justificativa de sua escolha.
