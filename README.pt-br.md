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

-   [docs/](docs/) - Documentação do projeto
    
-   [firmware/](firmware/) - Código fonte para microcontrolador
    
-   [software/](software/) - Aplicações para PC, scripts e simulações
    
-   [hardware/](hardware/) - Arquivos de design de hardware
    
    -   [electronics/](hardware/electronics/) - Arquivos de design eletrônico
        
        -   [kicad/](hardware/electronics/kicad/) - Esquemas e layouts de PCB (arquivos do projeto KiCad)
            
        -   [bom/](hardware/electronics/bom/) - Lista de Materiais (BOM)
            
    -   [mechanics/](hardware/mechanics/) - Modelos mecânicos (CAD, STL, STEP, desenhos)
        
-   [tests/](tests/) - Testes de integração (firmware + hardware + software)
    
-   [tools/](tools/) - Scripts utilitários e ferramentas auxiliares
    
-   [LICENSE](LICENSE) - Arquivo de licença
    
-   [README.md](README.md) - Este arquivo
  
## 📌 Referência

Para informações sobre a **motivação, objetivos de design e critérios de desenvolvimento** do robô, 
consulte o artigo *“Juca: an embedded mobile robotics board for education”*, 
publicado no *16th Workshop on Robotics in Education (WRE 2025)*
