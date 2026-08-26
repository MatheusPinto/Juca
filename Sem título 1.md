

# 1. Problema crítico com `127.0.0.1`

A documentação afirma que o site está hospedado no InfinityFree e que o arquivo `upload.php` acessa:

```text
http://127.0.0.1:1880/api/state
```

para buscar dados do Node-RED executado no computador da bancada.

Isso precisa ser esclarecido porque há duas possibilidades completamente diferentes.

## Caso a requisição seja feita pelo PHP

Se o código executado em `upload.php` fizer a requisição no lado do servidor, então:

```text
127.0.0.1 = servidor do InfinityFree
```

e não o computador da bancada.

O servidor hospedado na internet não consegue acessar o Node-RED do computador do aluno por meio de `127.0.0.1`. Nesse caso, a arquitetura descrita simplesmente não funciona.

## Caso a requisição seja feita por JavaScript no navegador

Se `upload.php` apenas entrega uma página que contém JavaScript, então:

```text
127.0.0.1 = computador no qual o navegador foi aberto
```

Nesse caso, pode funcionar somente quando:

- a página é aberta no mesmo computador que executa o Node-RED;
    
- o navegador permanece aberto;
    
- o Node-RED permite a requisição;
    
- as políticas de CORS e conteúdo misto não bloqueiam a comunicação.
    

A documentação mistura PHP, JavaScript e “o próprio computador” sem dizer **onde a requisição é executada**.

Esse é o primeiro ponto que precisa ser corrigido.

# 2. O diagrama de arquitetura é ambíguo

O fluxograma da página 5 apresenta:

- `index.php`;
    
- `upload.php`;
    
- JavaScript;
    
- `controller.php`;
    
- `backend.php`;
    
- MySQL.
    

Porém, ele não distingue claramente:

- navegador do usuário;
    
- servidor InfinityFree;
    
- computador da bancada;
    
- Node-RED;
    
- banco MySQL;
    
- Internet;
    
- rede local.
    

Essa separação é fundamental.

Um diagrama adequado deveria mostrar algo parecido com:

```text
Computador da bancada
┌────────────────────────────────┐
│ Node-RED :1880                 │
│ Navegador ou agente de envio   │
└──────────────┬─────────────────┘
               │ HTTPS
               ▼
Servidor InfinityFree
┌────────────────────────────────┐
│ endpoint PHP de recepção       │
│ banco de dados MySQL           │
│ dashboard PHP/JavaScript       │
└────────────────────────────────┘
```

A solução mais robusta seria o **Node-RED enviar os dados para o site**, e não o site hospedado tentar buscar dados em um endereço local.

# 3. Não está explicado quem executa periodicamente o `upload.php`

A documentação afirma que `upload.php` busca os dados e os insere no banco, mas não informa:

- quem chama esse arquivo;
    
- com qual periodicidade;
    
- se existe `setInterval`;
    
- se existe um nó no Node-RED;
    
- se existe uma tarefa agendada;
    
- se é necessário abrir manualmente a página;
    
- se a coleta continua quando nenhum navegador está aberto.
    

O texto apenas diz que é necessário acrescentar `/upload.php` à URL.

Isso sugere que alguém precisa abrir a página manualmente. Se for assim, a sincronização não é autônoma.

É necessário documentar explicitamente o fluxo:

```text
A cada X segundos:
1. origem solicita /api/state;
2. recebe o JSON;
3. envia o JSON ao backend;
4. backend valida os dados;
5. backend grava no banco;
6. dashboard consulta o registro mais recente.
```

# 4. A explicação sobre CORS contém erro conceitual

A seção de CORS afirma que, no campo `origin`, deve ser colocado:

> protocolo + domínio + porta, sendo apresentada a porta 1880 como padrão do Node-RED.

O valor de `origin` deve representar a **origem da página que está fazendo a requisição**, não o endereço nem a porta do servidor Node-RED.

Para um site como:

```text
https://curricularium.infinityfreeapp.com
```

a origem é:

```text
https://curricularium.infinityfreeapp.com
```

A porta `1880` pertence ao destino Node-RED e não deve ser acrescentada à origem do site, salvo se a própria página web estiver sendo servida nessa porta.

Além disso, CORS só é relevante quando a requisição é feita pelo navegador. Se a requisição for realizada pelo PHP no servidor, CORS não se aplica.

Portanto, a própria necessidade de CORS confirma que provavelmente existe JavaScript no navegador, mas isso não é explicado claramente.

# 5. Inconsistência entre `deviceId` e `deviceID`

O objeto padrão criado no Node-RED usa:

```javascript
deviceId: "NEXUS Central Node V2"
```

Mas o “objeto JSON esperado no Website” usa:

```json
"deviceID": "Nexus_Hub"
```

JavaScript e JSON diferenciam maiúsculas de minúsculas. Portanto:

```text
deviceId ≠ deviceID
```

Se o website procurar `deviceID`, mas a API retornar `deviceId`, o valor poderá aparecer como inexistente.

Também há divergência no próprio conteúdo:

```text
NEXUS Central Node V2
Nexus_Hub
```

É necessário definir um único contrato JSON.

# 6. Inconsistência entre `estado` e `status`

O objeto inicial MQTT contém:

```javascript
MQTT: {
    online: true,
    temperatura: "---",
    estado: "---"
}
```

Entretanto, a função que atualiza o estado grava:

```javascript
protocolState.MQTT = {
    ...
    status: String(msg.payload)
};
```

Assim, o objeto poderá terminar com os dois campos:

```json
{
  "estado": "---",
  "status": "Sistema aquecendo"
}
```

Se o website estiver lendo `estado`, continuará exibindo `---`, embora `status` tenha sido atualizado.

Esse é um erro funcional concreto. É preciso escolher um único nome:

```javascript
estado: String(msg.payload)
```

ou padronizar todo o sistema em inglês.

# 7. O sistema informa que os protocolos estão online sem verificar

O estado padrão define:

```javascript
PROFINET.online = true
CAN.online = true
MQTT.online = true
```

Além disso, as funções de atualização sempre escrevem `online: true`.

No final da documentação, admite-se que não houve tempo para implementar a verificação real de disponibilidade.

Isso significa que o dashboard pode apresentar uma rede como online mesmo quando:

- o dispositivo está desligado;
    
- o cabo foi desconectado;
    
- o broker MQTT caiu;
    
- o CLP não responde;
    
- nenhum dado é recebido há horas.
    

Esse comportamento é enganoso.

O padrão deveria ser:

```javascript
online: false
```

Cada protocolo também deveria manter:

```javascript
lastUpdate: Date.now()
```

e a API poderia calcular:

```javascript
online = Date.now() - lastUpdate < timeout
```

Por exemplo, um protocolo seria considerado offline se não houvesse atualização por mais de cinco segundos.

# 8. O estado está apenas na memória do Node-RED

O código usa:

```javascript
flow.get("protocolState")
flow.set("protocolState", protocolState)
```

Isso armazena o estado no contexto do fluxo. Porém, a documentação não informa se foi configurado armazenamento persistente do contexto.

Sem uma configuração persistente, quando o Node-RED reiniciar:

- todos os estados serão perdidos;
    
- os valores voltarão aos padrões;
    
- todos poderão aparecer incorretamente como online.
    

A documentação deveria informar se o `contextStorage` está configurado em memória ou em arquivo e qual comportamento é esperado após uma reinicialização.

# 9. Credenciais do banco estão gravadas diretamente no código

A documentação informa que `backend.php` possui os parâmetros:

- host;
    
- nome do banco;
    
- usuário;
    
- senha.
    

Na imagem da página 20, a classe `Connection` mostra essas informações diretamente no código. Mesmo que a senha esteja desfocada no PDF, o arquivo real do repositório pode conter o valor em texto simples.

Isso é um problema grave de segurança.

As credenciais não deveriam ser:

- publicadas no README;
    
- colocadas em capturas de tela;
    
- versionadas no Git;
    
- armazenadas dentro de arquivos públicos da pasta `htdocs`.
    

Medidas mínimas:

1. alterar imediatamente a senha exposta;
    
2. remover a credencial do histórico do Git;
    
3. utilizar arquivo de configuração fora da pasta pública;
    
4. incluir esse arquivo no `.gitignore`;
    
5. fornecer apenas um `config.example.php` sem valores reais.
    

Por exemplo:

```php
<?php
return [
    'host' => 'SEU_HOST',
    'database' => 'SEU_BANCO',
    'username' => 'SEU_USUARIO',
    'password' => 'SUA_SENHA'
];
```

# 10. O endpoint `upload.php` parece publicamente acessível

A documentação instrui acessar diretamente:

```text
dominio.infinityfreeapp.com/upload.php
```

Não é apresentada nenhuma forma de:

- autenticação;
    
- token;
    
- assinatura;
    
- restrição por método HTTP;
    
- proteção contra chamadas repetidas;
    
- validação da origem;
    
- limitação de frequência.
    

Se esse arquivo grava no banco de dados, qualquer visitante pode potencialmente acioná-lo.

CORS não protege o endpoint contra chamadas externas. CORS apenas controla se determinado JavaScript no navegador pode ler uma resposta. Um atacante ainda pode enviar requisições diretamente.

O endpoint de recebimento deveria exigir, no mínimo, um token secreto ou uma chave de API.

# 11. O banco de dados praticamente não é documentado

Existe um arquivo `database_export.sql`, mas a documentação não apresenta:

- nome das tabelas;
    
- colunas;
    
- tipos;
    
- chave primária;
    
- relacionamentos;
    
- campo de data e hora;
    
- índice;
    
- política de retenção;
    
- exemplo de registro;
    
- frequência das inserções.
    

O texto limita-se a dizer que as tabelas podem ser importadas pelo phpMyAdmin.

Para uma documentação técnica, deveria existir uma seção como:

|Campo|Tipo|Descrição|
|---|---|---|
|`id`|`BIGINT`|Identificador do registro|
|`timestamp`|`DATETIME`|Horário da amostra|
|`protocol`|`VARCHAR(20)`|CAN, MQTT ou PROFINET|
|`variable`|`VARCHAR(50)`|Nome da variável|
|`value`|`DECIMAL(...)`|Valor medido|

Também deveria ser explicado se cada atualização cria uma nova linha ou substitui o último estado.

# 12. Não é informado se são usadas consultas preparadas

A documentação diz que `backend.php` realiza consultas ao banco, mas não informa se utiliza:

- PDO;
    
- `mysqli`;
    
- consultas preparadas;
    
- validação de tipos;
    
- tratamento de exceções;
    
- transações.
    

Não é possível afirmar somente pelo README que existe uma vulnerabilidade de SQL injection, mas a ausência dessa informação é uma lacuna importante, principalmente porque o endpoint parece ser público.

# 13. Risco de exceder o limite da hospedagem

A documentação alerta para um limite de 50.000 requisições, mas não informa o período de atualização do dashboard nem da coleta.

Esse detalhe é essencial. Por exemplo:

- uma requisição por segundo representa 86.400 requisições por dia;
    
- uma requisição a cada cinco segundos representa 17.280 por dia;
    
- várias páginas abertas multiplicam esse consumo.
    

O documento precisa indicar:

- intervalo de atualização;
    
- quantidade de requisições por ciclo;
    
- estimativa diária;
    
- comportamento com múltiplos usuários;
    
- existência de cache.
    

# 14. Definições tecnicamente fracas ou incorretas

A seção que define HTML, CSS, JavaScript, PHP e MySQL apresenta problemas.

## JavaScript

> “Linguagem responsável por tornar o website responsive.”

JavaScript não é responsável necessariamente pela responsividade. Responsividade visual normalmente é implementada principalmente com CSS.

Melhor:

> JavaScript é uma linguagem utilizada para implementar comportamento dinâmico, manipulação do DOM e comunicação assíncrona com servidores.

## PHP

> “Suporta HTML e Javascript em seu arquivo.”

PHP é uma linguagem executada no servidor que pode gerar HTML, JSON ou outros conteúdos. Não é correto defini-lo apenas como algo que “suporta HTML e JavaScript”.

## MySQL

> “Estrutura semântica lida por banco de dados.”

MySQL não é uma estrutura semântica. É um sistema gerenciador de banco de dados relacional.

## HTML

O termo correto é **árvore DOM**, e não apenas “árvore de documentos/objetos”.

Essas definições introdutórias ocupam espaço, mas não ajudam a reproduzir o projeto. Seria melhor reduzir essa parte e ampliar a arquitetura, implantação e banco de dados.

# 15. Explicação incorreta sobre `DirectoryIndex`

A documentação afirma que o servidor procura um arquivo chamado `index` e “quaisquer variações de extensão”.

O Apache não tenta extensões arbitrárias. Ele procura os nomes configurados na diretiva `DirectoryIndex`, na ordem especificada.

Uma explicação correta seria:

> Quando o usuário acessa um diretório sem informar um arquivo, o Apache procura, na ordem configurada pela diretiva `DirectoryIndex`, por arquivos como `index.php` e `index.html`.

Além disso, o nome do arquivo foi escrito repetidamente como:

```text
.htacess
```

O correto é:

```text
.htaccess
```

# 16. Uso impreciso da palavra “upload”

O arquivo `upload.php` aparentemente não recebe um arquivo enviado pelo usuário. Ele coleta um estado JSON e o grava em um banco.

Assim, termos mais precisos seriam:

- sincronização;
    
- ingestão de dados;
    
- coleta;
    
- atualização;
    
- persistência.
    

“Upload” não é necessariamente incorreto em sentido amplo, mas pode levar o leitor a pensar em envio de arquivos.

# 17. API sem contrato formal

A documentação apresenta `/api/state`, mas não informa:

- método HTTP esperado;
    
- códigos de resposta;
    
- tipos dos campos;
    
- unidades;
    
- campos obrigatórios;
    
- tratamento de valores ausentes;
    
- autenticação;
    
- versão da API;
    
- exemplo de erro;
    
- frequência recomendada.
    

Ela mostra um exemplo de JSON, mas esse exemplo já diverge da implementação em `deviceId/deviceID` e `estado/status`.

Seria necessário definir um contrato único, por exemplo:

```json
{
  "schemaVersion": 1,
  "deviceId": "nexus-central",
  "timestamp": "2026-07-14T21:00:00-03:00",
  "profinet": {
    "online": true,
    "frequencyHz": 30.0
  },
  "can": {
    "online": true,
    "speedKmh": 12.5,
    "gear": 1,
    "errorCode": 0
  },
  "mqtt": {
    "online": true,
    "temperatureC": 24.5,
    "state": "heating"
  }
}
```

# 18. Problemas no tratamento do JSON CAN

Quando o código não consegue converter o conteúdo para JSON, ele executa:

```javascript
node.warn("Invalid CAN JSON");
return msg;
```

Ao retornar `msg`, a mensagem inválida pode continuar para os próximos nós. Para interromper o fluxo, o comportamento normalmente deveria ser:

```javascript
return null;
```

Também seria adequado registrar o conteúdo recebido e atualizar a condição de erro do protocolo.

# 19. Não há data e hora nos registros do estado

O JSON apresentado não possui:

- `timestamp`;
    
- sequência;
    
- idade do dado;
    
- horário da última atualização.
    

Sem isso, um valor de temperatura ou velocidade pode parecer atual mesmo sendo antigo.

Cada protocolo deveria armazenar pelo menos:

```javascript
lastUpdate: Date.now()
```

O banco de dados deveria registrar a data e hora de recebimento de cada amostra.

# 20. Problemas visuais da documentação

Há vários problemas de apresentação:

- a página 15 contém apenas uma captura do fluxo MQTT, praticamente sem explicação;
    
- as páginas 20 e 21 são intituladas “Fluxograma”, mas mostram capturas de configuração e banco de dados;
    
- algumas imagens ocupam páginas inteiras com muito espaço em branco;
    
- os trechos de código aparecem como capturas de tela, dificultando cópia, busca e manutenção;
    
- várias imagens têm resolução baixa;
    
- as legendas são genéricas;
    
- a página 18 expõe informações da conta e da infraestrutura de hospedagem.
    

É preferível utilizar blocos de código Markdown e diagramas exportados diretamente.

# 21. Problemas gramaticais recorrentes

Alguns exemplos:

|Original|Correção|
|---|---|
|`Javascript`|`JavaScript`|
|`Node-Red`|`Node-RED`|
|`.htacess`|`.htaccess`|
|`usuario`|`usuário`|
|`diretorio`|`diretório`|
|`circunstancia`|`circunstância`|
|`responsavel`|`responsável`|
|`requisições periodicas`|`requisições periódicas`|
|`contem`|`contém`|
|`icones`|`ícones`|
|`invisivel`|`invisível`|
|`Estrura semantica`|`estrutura semântica`|
|`realisar`|`realizar`|
|`parametros`|`parâmetros`|
|`auto explicativos`|`autoexplicativos`|
|`necessario`|`necessário`|
|`pderá`|`poderá`|
|`está opção`|`esta opção`|
|`proximo semestres`|`próximos semestres`|

Também há muitas frases iniciadas com letra minúscula e períodos excessivamente longos.

# Pontos positivos

Apesar dos problemas, existem aspectos úteis:

1. O repositório separa `htdocs`, figuras e exportação do banco.
    
2. A documentação registra o fluxo `/api/state`.
    
3. O objeto agrega informações das três células: PROFINET, CAN e MQTT.
    
4. Existe preocupação com continuidade para turmas futuras.
    
5. O arquivo de exportação do banco pode facilitar a reconstrução.
    
6. A interface mostrada na página 2 é visualmente simples e permite visualizar as três redes.
    
7. A documentação reconhece que a verificação real do estado online ainda está pendente, embora minimize indevidamente essa limitação.
    

# Correções prioritárias

## Críticas

1. Definir onde `upload.php` e a requisição a `127.0.0.1` são executados.
    
2. Substituir a busca do site pelo envio ativo do Node-RED para o servidor.
    
3. Remover e trocar imediatamente as credenciais publicadas.
    
4. Proteger o endpoint de gravação com autenticação.
    
5. Corrigir `deviceId/deviceID`.
    
6. Corrigir `estado/status`.
    
7. Fazer o estado online iniciar como falso e implementar tempo limite.
    
8. Adicionar `timestamp` aos dados.
    
9. Documentar como a sincronização é disparada.
    

## Importantes

1. Criar um diagrama de implantação com os limites de rede.
    
2. Documentar o esquema MySQL.
    
3. Explicar a periodicidade de atualização.
    
4. Padronizar a API.
    
5. Documentar a persistência do contexto Node-RED.
    
6. Corrigir a explicação de CORS.
    
7. Retirar dados reais da conta InfinityFree.
    
8. Informar instruções completas de implantação.
    

## Editoriais

1. Corrigir `.htaccess`.
    
2. Revisar toda a gramática.
    
3. Substituir capturas de código por blocos Markdown.
    
4. Corrigir legendas como “Fluxograma”.
    
5. Reduzir definições genéricas de HTML, CSS e PHP.
    
6. Padronizar nomes e capitalização.
    

## Conclusão

O projeto tem uma finalidade válida: disponibilizar na internet os estados consolidados do Node-RED. Porém, a documentação ainda não separa corretamente **execução no navegador**, **execução no servidor hospedado** e **execução no computador local**. Essa confusão aparece principalmente no uso de `127.0.0.1` e na explicação de CORS.

A arquitetura mais adequada seria:

```text
Node-RED local
    │
    │ POST HTTPS autenticado
    ▼
API PHP hospedada
    │
    ▼
MySQL
    │
    ▼
Dashboard público
```

Nesse modelo, o Node-RED envia os dados para o servidor, o servidor valida e armazena, e o dashboard apenas consulta o banco. Isso elimina a dependência de um navegador aberto na bancada e evita a tentativa de o servidor remoto acessar um `localhost` que não lhe pertence.