# robo-tcc
---

## Organização do Repositório

### Branchs
O Repositório está dividido em duas branchs (master e dev). Todo o código que está em desenvolvimento/teste encontra-se na branch dev e, todo o código estável encontra-se na branch master.

### Pastas

A divisão de pastas segue a seguinte estrutura:

config -> Arquivos de configuração do ROS.

core -> Código de controle de fluxo de informação.

install -> Instalador do código (converte a estrutura do projeto para a estrutura do ROS).

internal_server -> Servidor utilizado internamente para comunicação com os motores do robô.

modules -> Módulos extras do robô (controle das rodas, sensores, etc).

msg -> Protocólos de comunicação utilizados dentro do ROS.

server -> Servidor utilizado para comunicação com aplicações externas (controle e envio de missões).

services -> Serviços utilizados por outros códigos (logs, acesso à variáveis de ambiente).
