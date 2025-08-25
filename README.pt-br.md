[![en](https://img.shields.io/badge/lang-en-red.svg)](/README.md)
[![pt-br](https://img.shields.io/badge/lang-pt--br-green.svg)](/README.pt-br.md)

# Juca

Juca é uma placa de robótica móvel embarcada projetada especificamente para fins educacionais. 
Juca combina **custo acessível**, **expansibilidade** e **funcionalidade robusta**, 
tornando-a adequada tanto para ambientes de sala de aula quanto para competições. 
Entre suas principais características estão o **hardware modular semelhante aos shields do Arduino**, 
suporte para **robôs com tração diferencial** e **integração simplificada com plataformas educacionais de programação**.

## 📂 Estrutura do Repositório

Este repositório contém todos os recursos relacionados ao desenvolvimento do rover, 
incluindo **firmware**, **software** e **hardware** (eletrônica e mecânica).

Juca/
│
├── documentacao/           # Documentação do projeto
│
├── firmware/               # Código fonte para microcontrolador
│
├── software/               # Aplicações para PC, scripts e simulações
│
├── hardware/
│   ├── eletronica/
│   │   ├── kicad/         # Esquemas e layouts de PCB (arquivos do KiCad)
│   │   └── lista-materiais/ # Lista de Materiais (BOM)
│   │
│   └── mecanica/          # Modelos mecânicos (CAD, STL, STEP, desenhos)
│
├── testes/                 # Testes de integração (firmware + hardware + software)
│
├── ferramentas/            # Scripts utilitários e ferramentas auxiliares
│
├── LICENCA
└── README.md

## 📌 Referência

Para informações sobre a **motivação, objetivos de design e critérios de desenvolvimento** do robô, 
consulte o artigo *“Juca: an embedded mobile robotics board for education”*, 
publicado no *16th Workshop on Robotics in Education (WRE 2025)*