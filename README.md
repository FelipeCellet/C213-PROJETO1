# C213-PROJETO1
 Primerio projeto C213

 Explicação do Método Smith e Escolhas de Controle PID
1. Introdução
O método de Smith é amplamente utilizado para modelagem de sistemas com atraso de transporte. Nesse contexto, ele é eficaz para sistemas de primeira ordem com um atraso considerável, proporcionando uma modelagem precisa quando os dados experimentais possuem baixa incerteza. No projeto de controle utilizado, o método de Smith foi aplicado para estimar a função de transferência do sistema e ajustar o controlador PID de forma a obter um desempenho mais adequado.

2. Motivo da Escolha do Método Smith
O método de Smith foi escolhido por ser ideal para sistemas que apresentam as seguintes características:

Atraso de transporte significativo: Quando o atraso no sistema não pode ser ignorado, o método de Smith é uma escolha natural, pois lida diretamente com esse fenômeno.
Simplicidade e precisão: Ele permite calcular o ganho estático (k), a constante de tempo (τ) e o atraso de transporte (θ) com precisão, o que é crucial para a sintonização de controladores PID.
Aproximação de Padé: Utilizando a aproximação de Padé, o método de Smith representa bem o atraso de transporte em sistemas de controle, modelando adequadamente as dinâmicas de sistemas com atraso.
Essas razões tornam o método de Smith eficaz para sistemas de primeira ordem com atraso de transporte, como o utilizado no projeto.

3. Controladores PID Utilizados
Para garantir que o sistema tenha um bom desempenho tanto em malha aberta quanto em malha fechada, foram aplicados dois métodos de sintonização de controladores PID:

Ziegler-Nichols (Malha Aberta): Um método amplamente utilizado para sistemas com atraso e que foca na obtenção de respostas rápidas, mas pode gerar sobressinal (overshoot). Esse método foi escolhido por sua simplicidade e eficiência em muitos casos industriais.
CHR com Sobrevalor: Esse método é uma alternativa ao Ziegler-Nichols, com o objetivo de reduzir o overshoot e proporcionar uma resposta mais suave. Ele oferece um compromisso entre o tempo de resposta e a estabilidade do sistema.
Esses dois métodos de sintonização foram aplicados e comparados, para observar o comportamento do sistema controlado em termos de tempo de subida, tempo de acomodação e erro estacionário.

4. Resultados e Desempenho do Sistema
As simulações mostraram as seguintes características importantes:

Malha Aberta: O sistema, sem controle, apresenta um tempo de subida maior, assim como um tempo de acomodação mais longo. O erro estacionário é significativo, o que indica a necessidade de um controlador.
Malha Fechada (Ziegler-Nichols): O sistema responde mais rapidamente, mas apresenta overshoot (sobressinal). O tempo de acomodação é menor, mas o overshoot pode ser indesejável em alguns cenários.
Malha Fechada (CHR): Apresenta uma resposta mais equilibrada, com menor overshoot e bom tempo de acomodação. Essa é uma boa opção quando se busca um compromisso entre velocidade e estabilidade.
5. Considerações Finais
Ao escolher um método de controle para sistemas com atraso, é essencial levar em consideração as características do sistema e os objetivos do controle. No caso do método de Smith, ele se mostrou eficiente por lidar diretamente com o atraso de transporte, o que é uma característica marcante do sistema em análise.

Além disso, a escolha entre Ziegler-Nichols e CHR com sobrevalor para a sintonização do PID depende das exigências de desempenho. O Ziegler-Nichols pode ser preferível em sistemas que podem tolerar algum overshoot em troca de tempos de resposta mais rápidos, enquanto o CHR com sobrevalor proporciona uma resposta mais equilibrada, com menos overshoot e maior estabilidade.

Essas escolhas proporcionam flexibilidade no ajuste do sistema, permitindo que ele seja ajustado para diferentes aplicações e condições operacionais.
