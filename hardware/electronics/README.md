# Projeto Eletrônico do Juca

  
O Juca é a placa eletrônica propriamente dita, conforme observado na [Figura 1](#juca_board) e [Figura 2](#juca_board_back). Ela possui 180 mm x 125 mm, composta de duas camadas, com objetivo de ser robusta. 


<div id="juca_board" align="center">
  <img src="figs/juca_board.png" alt="Visão 3D da parte superior da placa Juca." width="700">
  <br>
  <i>Figura 1: Visão 3D da parte superior da placa Juca.</i>
</div>

<br><br>

<div id="juca_board_back" align="center">
  <img src="figs/juca_board_back.png" alt="Visão 3D da parte inferior da placa Juca." width="700">
  <br>
  <i>Figura 2: Visão 3D da parte inferior da placa Juca.</i>
</div>

<br><br>

Na [Figura 3](#juca_diagram) é apresentado o diagrama de blocos do Juca. Serão apresentados cada módulo presente no sistema.

<br>

<div id="juca_diagram" align="center">
  <img src="figs/juca_diagram.png" alt=" Diagrama de blocos didático do sistema." width="600">
  <br>
  <i>Figura 3: Diagrama de blocos didático do sistema.</i>
</div>

---

## Microcontrolador (SoC)

O sistema utiliza o SoC **ESP32-S3-WROOM-1**, aproveitando suas capacidades de processamento dual-core e capacidade de comunicação por Wi-fi e BLE. O esquemático relacionado a esse módulo encontra-se na [Figura 4](#esp32_s3_sch).
O ESP32-S3 têm picos de consumo de corrente muito rápidos (especialmente ao ligar o Wi-Fi). Os capacitores (C11 e C12) próximos aos pinos de alimentação (`3V3`) filtram ruídos de alta frequência e evitam que a tensão caia momentaneamente, o que causaria resets inesperados.

<br>

<div id="esp32_s3_sch" align="center">
  <img src="figs/esp32_s3_sch.png" alt="Esquemático módulo ESP32-S3-WROOM-1." width="600">
  <br>
  <i>Figura 4: Esquemático módulo ESP32-S3-WROOM-1.</i>
</div>

<br><br>

- **R13 (10k Pull-up no EN):** Mantém o pino de Enable (Reset) em nível alto (3.3V) para que o chip opere de maneira normal quando o pushbutton SW1 não for pressionado. O botão SW1, ao ser pressionado, aterra esse pino para resetar a MCU.

- **R14 (10k Pull-down no GPIO45):** Este é um **resistor de configuração (Strapping Pin)**. Ele garante que o pino 45 esteja em 0V durante o boot, informando ao ESP32-S3 que a memória Flash interna opera a **3.3V**. Sem ele, o chip poderia tentar alimentar a memória com 1.8V, impedindo a inicialização.

<br>

Toda gravação de firmware e comunicação com o chip é feita diretamente por meio da interface USB implementada diretamente no ESP32-S3 ([Figura 5](#usb_sch)), através dos pinos GPIO19 (DEBUG_DM) e GPIO20 (DEBUG_DP).

<br>

<div id="usb_sch" align="center">
  <img src="figs/usb_sch.png" alt="Interface USB de conexão com o ESP32-S3." width="400">
  <br>
  <i>Figura 5: Interface USB de conexão com o ESP32-S3.</i>
</div>

---

## Alimentação

O hardware possui três barramentos principais de alimentação ([Figura 7](#power_sch)):

1.  **+BATT (Potência):** 6V a 12V para os motores e alimentação geral do sistema. Essa alimentação pode advir de uma bateria (por meio do conector J3) ou de uma fonte CC externa conectada por um por jack-cc (J4). Um interruptor (SW3) é responsável por conectar a bateria/fonte ao restante da placa. Se bateria e fonte estiver conectadas no sistema, a fonte externa poderá carregar a bateria. Note que a placa não possui BMS, dessa forma será necessário utilizar no próprio pack de bateria um BMS.

2. **5V (Lógica):** Gerado por um conversor *buck* bastante utilizado em projetos de eletrônica hobistas (Figura) ou pela interface USB ([Figura 6](#buck_il_2825)). Cria uma barramento de 5V para alimentar dispositivos externos, e para alimentar o driver de motor e o sensores ultrassônicos. Se alimentação vier do USB (sem fonte +BATT), o driver de motor é acionado, mas os motores não.

<div id="buck_il_2825" align="center">
  <img src="figs/buck_il_2825.jpg" alt="Mini placa para buck IL 2825." width="300">
  <br>
  <i>Figura 6: Mini placa para buck IL 2825.</i>
</div>

<br>

3.  **3.3V (Lógica):** Gerado por um regulador LDO (como o AMS1117) para alimentar o ESP32-S3 e o restantes dos sensores.

<br>

<div id="power_sch" align="center">
  <img src="figs/power_sch.png" alt="Módulos de alimentação da placa." width="600">
  <br>
  <i>Figura 7: Módulos de alimentação da placa.</i>
</div>


---
## *Driver* de motores

Para acionamento dos motores por meio do PWM, utilizou-se o chip L298N. Esse módulo possui uma ponte H para controle de direção e amplificação da potência nos motores. Um diagrama do circuito do L298N retirado do seu datasheet é apresentado na [Figura 8](#l298N). O esquemático do driver desenvolvido com o chip é apresentado na [Figura 9](#mot_driver_sch).

<br>

<div id="l298N" align="center">
  <img src="figs/l298N.png" alt="Diagrama do circuito interno do L298N." width="600">
  <br>
  <i>Figura 8: Diagrama do circuito interno do L298N.</i>
</div>

<br><br>

<div id="mot_driver_sch" align="center">
  <img src="figs/mot_driver_sch.png" alt="Esquemático do circuito *driver* de motor do Juca." width="900">
  <br>
  <i>Figura 9: Esquemático do circuito driver de motor do Juca.</i>
</div>

<br><br>

A placa foi pensada usando um motor com as seguintes especificações (mas facilmente adaptado para outros tipos de motor, mudando valores de tensão de alimentação da bateria e/ou mudando alguns valores de resistores no circuito de sensor de corrente):

|**Parâmetro**|**Valor / Especificação**|
|---|---|
|**Tensão de Operação**|6V DC|
|**Velocidade (Sem Carga)**|280 RPM|
|**Máxima Eficiência**|2.0 kg.cm @ 170 RPM|
|**Potência na Máx. Eficiência**|2.0 W|
|**Corrente na Máx. Eficiência**|0.60 A|
|**Torque de Partida (Stall)**|5.2 kg.cm @ 110 RPM|
|**Potência Máxima**|3.1 W|
|**Corrente de Pico (Stall)**|1.10 A|
|**Diâmetro do Motor**|25 mm|
|**Comprimento Total (Corpo + Eixo)**|73 mm|
|**Diâmetro do Eixo**|4 mm|
|**Comprimento do Eixo**|12 mm|

<br>

O motor utilizado a priori é daqueles se acham facilmente em lojas online de materiais eletrônicos, possuindo integrado um *encoder* de quadratura a efeito hall, conforme [Figura 10](#motor_wire)

<br>

<div id="motor_wire" align="center">
  <img src="figs/motor_wire.jpg" alt="Motor com encoder utilizado." width="600">
  <br>
  <i>Figura 10: Motor com encoder utilizado.</i>
</div>

<br>

Com base nas combinações dos pinos MOT_IN1, MOT_IN2, MOT_IN3 e MOT_IN4, pode-se determinar o controle de direção e velocidades das rodas. Note que quando todos os pinos estiverem em ALTO, as rodas irão travar eletronicamente, pois as portas lógicas SUPERIORES do driver ([Figura 8](#l298N)) irão forçar os polos do motor ao mesmo potencial elétrico (Vcc). Da mesma forma, quando todos os pinos estiverem em BAIXO, as rodas irão travar eletronicamente, pois as portas lógicas INFERIORES do driver ([Figura 8](#l298N)) irão forçar os polos do motor ao mesmo potencial elétrico (GND). Infelizmente, nesse projeto não há possibilidade de deixar as rodas em ponto morto, sem torque (deixar as rodas desconectadas), pois os pinos de habilitação (EnA e EnB) foram deixadas diretamente conectadas do +3V3.

<br>

| **Direção Desejada**  | **MOT_IN1** | **MOT_IN2** | **MOT_IN3** | **MOT_IN4** | **Efeito Prático**                       |
| --------------------- | ----------- | ----------- | ----------- | ----------- | ---------------------------------------- |
| **Frente**            | PWM         | Low         | PWM         | Low         | Ambas as rodas giram para frente         |
| **Trás**              | Low         | PWM         | Low         | PWM         | Ambas as rodas giram para trás           |
| **Girar p/ Direita**  | PWM         | Low         | Low         | PWM         | Roda esquerda p/ frente, direita p/ trás |
| **Girar p/ Esquerda** | Low         | PWM         | PWM         | Low         | Roda direita p/ frente, esquerda p/ trás |
| **Curva p/ Direita**  | PWM         | Low         | Low         | Low         | Apenas roda esquerda gira                |
| **Curva p/ Esquerda** | Low         | Low         | PWM         | Low         | Apenas roda direita gira                 |
| **Freio (Ativo)**     | Low         | Low         | Low         | Low         | Motores travam eletronicamente           |
| **Freio (Ativo)**     | High        | High        | High        | High        | Motores travam eletronicamente           |

<br>

Além do acionamento dos motores, o módulo de driver de motores fornece a corrente dos motores por meio de dois pinos SENSE_A e SENSE_B. Essas correntes são aplicadas a resistores de *shunt* de 0,1 Ohm PTH do tipo filme de carbono de 2 Watts (no limiar de potência para os motores), por serem fáceis de serem achadas comercialmente no Brasil. Sabendo a resistência dos *shunts* e medindo a tensão dos mesmos por meio de do ADC no SoC, aplica-se a Lei de Ohm e se obtém a corrente.. Posto isto, os valores de queda de tensão que surgem nesses resistores são muito pequenos, devendo ser necessário um circuito de amplificação (que será abordado posteriormente). 

O módulo também possui uma interface para entrada de encoders de quadratura. Essa parte será abordada mais adiante também, pois também serão apresentados os encoders utilizados. 


---
## Sensor de corrente dos motores

Para monitorar a carga dos motores e detectar obstáculos ou travamentos (stall), o projeto utiliza um circuito de amplificação baseado no AmpOp **LM324A** ([Figura 11](#curr_sense_sch)). O circuito utiliza a queda de tensão sobre os resistores de shunt (R1​ e R2​ de 0.1Ω em [Figura 9](#mot_driver_sch)) localizados nos pinos de _Sense_ do driver L298N. Como essa tensão é muito baixa para uma leitura precisa pelo ADC, aplicamos um **Amplificador Não-Inversor** a cada uma delas.

<br>

<div id="curr_sense_sch" align="center">
  <img src="figs/curr_sense_sch.png" alt="Esquemático do circuito de medição de corrente dos motores." width="900">
  <br>
  <i>Figura 11: Esquemático do circuito de medição de corrente dos motores.</i>
</div>

<br><br>

O ganho dos amplificadores foi determinado de tal forma que a corrente máxima de torque dos motores que foram utilizados (1,1 A) fosse próxima dos 3 V, já que o ADC do ESP32-S3 possui fundo de escala de 3V3. Como o LM324 não é rail-to-rail, ele satura cerca de 1.5V abaixo do VCC​, com 5V de alimentação, a saída máxima real será de aproximadamente **3.5V**. o suficiente para a aplicação. Para tanto, o cálculo usando o amplificador não inversor foi:

$$G=1+R_{in}​.R_{feedback​}​=1+3k.82k​≈28,33.$$

<br>
Abaixo está uma tabela ilustrativa com exemplos de tensão e leitura do ADC.
<br>

| **Corrente do Motor** | **Tensão no Shunt** | **Saída do AmpOp** | **Valor no ADC (10-bit)** | **Status do Motor** |
| --------------------- | ------------------- | ------------------ | ------------------------- | ------------------- |
| $0.1\text{A}$         | $0.01\text{V}$      | $0.28\text{V}$     | $\approx 87$              | Sem carga           |
| $0.6\text{A}$         | $0.06\text{V}$      | $1.70\text{V}$     | $\approx 527$             | Eficiência Máxima   |
| **$1.1\text{A}$**     | **$0.11\text{V}$**  | **$3.11\text{V}$** | **$\approx 965$**         | **Stall (Travado)** |
<br>
Na saída do amplificador é colocado um filtro passa-baixa RC (150Ω e 1μF) para eliminar ruídos de chaveamento (PWM), e um diodo Zener de 3.3V para garantir que a saída nunca ultrapasse o limite do pino analógico do microcontrolador.

---
## *Encoders*

O Juca é capaz de receber os dois sinais advindos de um encoder incremental de quadratura. Isso é importante no cálculo de odometria do robô, onde é possível determinar o deslocamento angular da roda com base nos pulsos dos sinais gerados pelo encoder.

O encoder é acoplado à roda, que possui algum tipo de disco magnético (ou óptico) multi-polar que gira solidário ao eixo do motor.

Os sinais gerados pelo encoder são geralmente denominados Fase A e Fase B. De forma básica, existem dois sensores que medem a variação do campo magnético (ou a passagem de luz) e geram ondas quadradas defasadas entre si em 90°. Essa defasagem é o que permite ao sistema identificar não apenas a velocidade, mas também o sentido de rotação (horário ou anti-horário), dependendo de qual fase sofre a transição de nível primeiro (ver [Figura 12](#quad_enc_anima) e [Figura 13](#encoder_wave)).

<br><br>


<div id="quad_enc_anima" align="center">
  <img src="figs/quad_enc_anima.gif" alt="Ilustração do comportamento de um encoder de quadratura." width="500">
  <br>
  <i>Figura 12: Ilustração do comportamento de um encoder de quadratura.</i>
</div>

<br>

<div id="encoder_wave" align="center">
  <img src="figs/encoder_wave.png" alt="Comportamento do encoder para o sentido direto e reverso da roda." width="500">
  <br>
  <i>Figura 13: Comportamento do encoder para o sentido direto e reverso da roda.</i>
</div>

<br>
<br>

O encoder utilizado primariamente no Juca é o do motor apresentado na [Figura 10](#motor_wire), e possui os seguintes parâmetros:

| **Parâmetro**            | **Detalhe / Valor**                      |
| ------------------------ | ---------------------------------------- |
| **Relação de Redução**   | 1:34 (Exato: 1:34.02)                    |
| **Resolução do Encoder** | 341.2 PPR (Pulsos por Revolução da Roda) |
| **Cálculo da Resolução** | 11 pulsos (motor) x 34.02 (redução)      |
| **Comprimento Total**    | 73 mm (incluindo motor e eixo)           |

O motor elétrico gira muito rápido, mas com pouca força (torque). Por isso, ele possui uma **caixa de engrenagens** acoplada.

- **O que significa:** O motor interno precisa dar **34,02 voltas** completas para que a roda do robô dê apenas **1 volta**.
- **Vantagem:** Isso aumenta o torque do motor em 34 vezes, permitindo que o robô carregue peso e vença a inércia, além de reduzir a velocidade para algo controlável (280 RPM).

O encoder está montado no eixo do motor (antes da redução). O sensor lê **11 pulsos** a cada volta do motor interno.

- **O cálculo:** Como a roda só gira uma vez a cada 34,02 voltas do motor, multiplicamos: 11×34,02=341,22.
- **PPR (Pulsos por Revolução):** Este é o número que seu código usa para saber que o robô completou uma volta de roda. Se o microcontrolador contou 341 pulsos, o robô percorreu quase que exatamente uma distância igual à circunferência da sua roda.

Para saber como o ESP32-S3 trabalha com os pulsos de encoders acesse este conteúdo.

---
## Sensores de linha IR

O módulo de sensores de linha IR não está presente diretamente na placa, devido às demandas de restrições geométricas necessárias para acomoda-la (geralmente longe da placa e em posições diversas de acordo com a necessidade do robô). Dessa forma, o Juca oferece uma interface para 5 sensores IR ([Figura 15](#ir_line_sch)).

<br>

<div id="ir_line_sch" align="center">
  <img src="figs/ir_line_sch.png" alt="Jumpers para interface com sensores de linha IR." width="300">
  <br>
  <i>Figura 14: Jumpers para interface com sensores de linha IR.</i>
</div>

<br>

Inicialmente utilizamos o módulo apresentado na [Figura 15](#ir_module). 

<br>

<div id="ir_module" align="center">
  <img src="figs/ir_module.png" alt="Módulo IR utilizado no Juca." width="400">
  <br>
  <i>Figura 15: Módulo IR utilizado no Juca.</i>
</div>

<br>

Na [Figura 16](#juca_ir_sensor) é possível visualizar como este módulo pode ser alocado na parte inferior de um possível chassi para o Juca, e quais posições podem ser escolhidas para cada pino de forma a detectar onde está posicionada uma linha no chão.

<br>

<div id="juca_ir_sensor" align="center">
  <img src="figs/juca_ir_sensor.png" alt="Desenho representativo do módulo IR abaixo de um chassi hipotético para o Juca e possíveis posições de detecção e pinos utilizados." width="400">
  <br>
  <i>Figura 16: Desenho representativo do módulo IR abaixo de um chassi hipotético para o Juca e possíveis posições de detecção e pinos utilizados.</i>
</div>

<br>

Para esse módulo é recomendada uma largura de linha de aproximadamente 20 mm.


---
## Sensores de distância ultrassônicos

O Juca possui uma interface para conexão com três dispositivos de detecção de obstáculos por ondas ultrassônicas. A interface criada pode ser observada no esquemático da [Figura 16](#hc_sr04_sch).


<br>

<div id="hc_sr04_sch" align="center">
  <img src="figs/hc_sr04_sch.png" alt="Módulo HC-SR04." width="500">
  <br>
  <i>Figura 17: Esquemático da interface de conexão com os módulos ultrassônicos.</i>
</div>

<br>

Os dispositivos ultrassônicos pensados para serem utilizados no Juca são do modelo HC-SR04 ([Figura 17](#hc_sr04)). Apesar disso, existem outros modelos no mercado com a mesma interface, que poderiam ser utilizadas no lugar. O ângulo de alcance de cada  HC-SR04 é aproximadamente 15 graus, tomando como origem o centro do transdutor de ondas.

<br>

<div id="hc_sr04" align="center">
  <img src="figs/hc_sr04.png" alt="Módulo HC-SR04." width="300">
  <br>
  <i>Figura 18: Módulo HC-SR04.</i>
</div>

<br>


Conforme pode ser observado na [Figura 18](#ultrasonic_detect), além da alimentação, o HC-SR04 possui um pino de gatilho e um pino de eco. O pino de gatilho tem por função a recepção de um pulso com duração aproximada de 10 𝜇s gerada pelo microcontrolador por meio de GPIO. Após geração do pulso, uma onda ultrassônica de 40 kHz é liberada pelo atuador por um período de 200 𝜇s. A presença de um objeto no caminho da onda irá refleti-la como um eco na direção do sensor. No pino de eco será gerado um pulso com largura $T_{onda}$ igual ao intervalo entre geração da onda e captação do eco. Este pulso irá variar entre 150 𝜇s e 25 ms caso um objeto seja detectado, sendo 38 ms caso nenhum obstáculo seja atingido. A distância 𝑑 do transdutor ao obstáculo é dada pela seguinte equação:

$$\begin{equation}
d = \frac{v_{som} \cdot T_{onda}}{2} 
\label{eq_ultrasonic}
\end{equation}$$

onde $v_{som}$ corresponde à velocidade do som no meio. Note que $T_{onda}/2$ corresponde ao tempo necessário para que a onda percorra o caminho entre o transdutor e o objeto – tempo relativo ao sinal gerado atingir o objeto, ou do eco atingir o sensor. Considerando como meio o ar em temperatura próxima aos 20 graus C, então  $v_{som}$ = 343 𝑚/𝑠 (HALLIDAY et al., 2013). Dessa forma, a distância do objeto pode variar entre 2,6 cm e 430 cm. Como a tensão nominal do HC-SR04 é 5 V, ele aceita sinal de 3V3 no pino de gatilho, mas o sinal gerado pelo eco é 5V. Por isso a interface no Juca utiliza um conversor de nível lógico (*level_shifter*).

<br>

<div id="ultrasonic_detect" align="center">
  <img src="figs/ultrasonic_detect.png" alt="Ilustração do comportamento de detecção de objetos com o HC-SR04." width="500">
  <br>
  <i>Figura 19: Ilustração do comportamento de detecção de objetos com o HC-SR04.</i>
</div>

<br>


---
## IMU

Os encoders são adequados em baixas frequências (estáticos/movimentos lentos). Uma Unidade de Medição Inercial - *Inertial Measurement Unit* (IMU) é adequada para capturar mudanças bruscas e rápidas (alta frequência). Ao aplicar um filtro EKF podemos combinar as duas fontes de dados, filtrando o ruído individual de cada sensor para gerar uma estimativa de pose muito mais robusta.

Uma IMU é composta basicamente por dois sensores independentes: um acelerômetro (mede  aceleração linear) e um giroscópio (mede aceleração angular). O IMU utilizado no Juca é o MPU-6050, possuindo 6 graus de liberdade (6 DoF). Isso quer dizer quer ele é capaz de medir aceleração linear em 3 direções (XYZ) e angular em 3 direções (XYZ). Foi utilizado um módulo pronto para o MPU-6050, denominado frequentemente de ITG/MPU. Uma foto dessa placa, com uma representação dos eixos coordenados no corpo e o sentido do ângulos, podem ser observados na [Figura 20](#mpu6050_gyro).

<br>

<div id="mpu6050_gyro" align="center">
  <img src="figs/mpu6050_gyro.jpg" alt="Uma foto a placa ITG/MPU (MPU-6050), com uma representação dos eixos coordenados no corpo e o sentido do ângulos." width="300">
  <br>
  <i>Figura 20: Uma foto a placa ITG/MPU (MPU-6050), com uma representação dos eixos coordenados no corpo e o sentido do ângulos.</i>
</div>

<br>

No Juca, o CI está localizado dentro da área do módulo de expansão ([Figura 1](#juca_board)). Os pinos utilizados podem ser visualizados na [Figura 21](#imu_sch).

<br>

<div id="imu_sch" align="center">
  <img src="figs/imu_sch.png" alt="Esquemático do MPU-6050 no Juca." width="300">
  <br>
  <i>Figura 21: Esquemático do MPU-6050 no Juca.</i>
</div>

<br>

Seguem algumas especificações pertinentes:

|**Categoria**|**Parâmetro**|**Detalhes / Valor**|
|---|---|---|
|**Alimentação**|Tensão de Operação (VCC)|**3.3V** (Conforme indicado no pino 8)|
|**Comunicação**|Protocolo de Interface|**I2C** (Pinos SCL e SDA) compartilhados nos pinos de expansão|
|**Pinagem (I2C)**|Clock / Dados|**Pino 6 (SCL)** e **Pino 5 (SDA)**|
|**Interrupção**|Sinal de Saída|**Pino 1 (INT)** - Conectado à linha `IMU_INT`|
|**Resolução**|ADC (Conversor Analógico-Digital)|**16 bits** por canal|
|**Giroscópio**|Fundo de Escala (Configurado)|**500°/s** (Escalas de ±250, 500, 1000, 2000°/s)|
|**Acelerômetro**|Eixos de Medição|**3 eixos** (X, Y, Z)|
|**Endereço I2C**|Seleção de Endereço (AD0)|**Pino 2 (AD0)** - Não utilizados (NC)|
|**Expansão**|Interface Auxiliar|**Pinos 3 (XCL) e 4 (XDA)** - Não utilizados (NC)|


---
## Interface de expansão



---
## Mapeamento de pinos (Pinout)

### Controle de motores, *encoders* e sensores de corrente

| Recurso         | Pino (GPIO) | Tipo/Interface  | Descrição                            |
| --------------- | ----------- | --------------- | ------------------------------------ |
| **MOT_IN1**     | GPIO11      | Saída (PWM)     | Motor esquerdo - sentido A           |
| **MOT_IN2**     | GPIO10      | Saída (PWM)     | Motor esquerdo - sentido B           |
| **MOT_IN3**     | GPIO13      | Saída (PWM)     | Motor direito - sentido A            |
| **MOT_IN4**     | GPIO12      | Saída (PWM)     | Motor direito - sentido B            |
| **ENC1_A**      | GPIO7       | Entrada PCNT    | Encoder motor direito - canal A      |
| **ENC1_B**      | GPIO6       | Entrada PCNT    | Encoder motor direito - canal B      |
| **ENC2_A**      | GPIO14      | Entrada PCNT    | Encoder motor esquerdo - canal A     |
| **ENC2_B**      | GPIO21      | Entrada PCNT    | Encoder motor esquerdo - canal B     |
| **CURR_LM_ADC** | GPIO2       | Analógico (ADC) | Leitura de corrente - motor esquerdo |
| **CURR_RM_ADC** | GPIO9       | Analógico (ADC) | Leitura de corrente - motor direito  |
### Sensores ultrassônicos

| Recurso         | Pino (GPIO) | Tipo/Interface               | Descrição                                                 |
| --------------- | ----------- | ---------------------------- | --------------------------------------------------------- |
| **SONAR_TRIG**  | GPIO42      | Saída Digital                | Gatilho para todos os sensores ultrassônicos (barramento) |
| **SONAR0_ECHO** | GPIO1       | Entrada (captura de entrada) | Pino de eco do sensor ultra. dir.                         |
| **SONAR1_ECHO** | GPIO4       | Entrada (captura de entrada) | Pino de eco do sensor ultra. centro                       |
| **SONAR2_ECHO** | GPIO5       | Entrada (captura de entrada) | Pino de eco do sensor ultra. esq.                         |
### Sensores de linha IR

| Recurso             | Pino (GPIO) | Tipo/Interface | Descrição                               |
| ------------------- | ----------- | -------------- | --------------------------------------- |
| **IR_LEFT**         | GPIO25      | Digital        | Sensor de Linha / Obstáculo esquerda    |
| **IR_CENTER_LEFT**  | GPIO33      | Digital        | Sensor de Linha / Obstáculo centro esq. |
| **IR_MIDDLE**       | GPIO32      | Digital        | Sensor de Linha / Obstáculo centro      |
| **IR_CENTER_RIGHT** | GPIO35      | Digital        | Sensor de Linha / Obstáculo centro dir. |
| **IR_RIGHT**        | GPIO34      | Digital        | Sensor de Linha / Obstáculo direita     |

### IMU

| Recurso     | Pino (GPIO) | Tipo/Interface | Descrição                             |
| ----------- | ----------- | -------------- | ------------------------------------- |
| **MCU_SDA** | GPIO40      | I2C            | Dados Barramento I2C (IMU / Expansão) |
| **MCU_SCL** | GPIO39      | I2C            | Clock Barramento I2C (IMU / Expansão) |
| **IMU_INT** | GPIO41      | Entrada/IRQ    | Interrupção da IMU (MPU-6050)         |

### Expansão (pinos sobressalentes para conexão de outros módulos externos)

| Pino (GPIO)  | Pino (GPIO) | Pino (GPIO) | Pino (GPIO) | Pino (GPIO)  |
| ------------ | ----------- | ----------- | ----------- | ------------ |
| GPIO3        | GPIO35      | GPIO36      | GPIO37      | GPIO38       |
| GPIO45       | GPIO46      | GPIO47      | GPIO48      | SDA (GPIO40) |
| SCL (GPIO39) | TXD0        | RXD0        |             |              |


---

  