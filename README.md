# Navegação e Mapeamento com ROS2

Este projeto visa a aplicação de **algoritmos de grafos** em um desafio de **navegação autônoma de labirinto** dentro de um simulador ROS 2 (`culling_games`). O desafio é dividido em duas partes principais:

1.  **Navegação Otimizada (Com Mapa):** Desenvolver um algoritmo de busca (e.g., BFS) para encontrar a rota mais curta do início ao alvo, tendo acesso prévio ao mapa completo (via serviço `/get_map`).
2.  **Mapeamento e Exploração (Sem Mapa):** O robô deve navegar pelo labirinto usando apenas **sensores locais** (`/culling_games/robot_sensors`) para mapeá-lo dinamicamente. Após encontrar o alvo e calcular o caminho ótimo nesse mapa construído, ele comprova a otimização executando o percurso.

## Estrutura do Código

O código do explorador está contido no pacote `navegacao`, que interage com o simulador `cg` (Culling Games).

### Localização dos Arquivos

Os arquivos principais estão localizados em:
`./src/navegacao/src/`

**Classes Principais:**

| Arquivo | Função |
| :--- | :--- |
| `ExploradorLabirinto.cpp/.h` | Nó principal. Implementa o ciclo de exploração, mapeamento, cálculo da rota ótima e execução otimizada. |
| `Algoritmo.cpp/.h` | Contém a lógica de busca em largura (BFS) e outras estratégias de navegação. |
| `Grafo.cpp/.h` | Representa a estrutura de dados do labirinto explorado como um grafo. |
| `ControladorRobo.cpp/.h` | Manipula as chamadas de serviço (`/move_command`, `/reset`) e a subscrição de tópicos. |


## Comunicação ROS2

A comunicação com o simulador é feita através dos seguintes tópicos e serviços:

### Tópico de Entrada (Sensores e Dados do Robô)

| Tópico | Tipo | Uso |
| :--- | :--- | :--- |
| **`/culling_games/robot_sensors`** | `cg_interfaces/msg/RobotSensors` | Recebe dados dos arredores imediatos (8 direções) do robô, usados para o mapeamento dinâmico. |
| `/parameter_events` | | Eventos de configuração do ROS 2. |
| `/rosout` | | Mensagens de log do sistema ROS 2. |

### Serviços de Controle (Comandos e Mapa)

| Serviço | Tipo | Uso |
| :--- | :--- | :--- |
| **`/move_command`** | `cg_interfaces/srv/MoveCmd` | Utilizado para enviar comandos de movimento cardinais (`up`, `down`, `left`, `right`) ao robô. |
| **`/get_map`** | `cg_interfaces/srv/GetMap` | Utilizado para obter a representação completa do labirinto (útil para a Parte 1 do desafio). |
| **`/reset`** | `cg_interfaces/srv/Reset` | Usado para reiniciar o jogo na mesma configuração de labirinto (`is_random: false`) ou em um novo. |

## Execução

Para compilar e executar o explorador, siga os passos abaixo:

### 1\. Compilação do Workspace

Na raiz do seu workspace (e.g., onde estão `build`, `install`, `src`), execute:

```bash
colcon build
```

### 2\. Configuração do Ambiente

Carregue as variáveis de ambiente do ROS2:

```bash
source install/setup.bash
```

### 3\. Iniciar o Simulador

Em um terminal, inicie o nó (ele inia automaticamente o labirinto):

```bash
ros2 run navegacao grafo_node
```

## 📺 Vídeo Demonstrativo

[Link](https://drive.google.com/file/d/17oOg_b2TlWxuKg0cROLFN5qsK4VAfUQ5/view?usp=sharing).
