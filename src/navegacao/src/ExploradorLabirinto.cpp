#include "ExploradorLabirinto.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <climits>
#include <stack>

// Adicione este método privado no header (.h):
// std::stack<PosicaoRobo> pilhaCaminho;  // Pilha para backtracking
// Adicione na classe como membro privado

ExploradorLabirinto::ExploradorLabirinto() 
    : Node("explorador_labirinto"),
      posicaoAtual{0, 0},
      posicaoInicial{0, 0},
      alvoEncontrado(false),
      movimentoEmAndamento(false),
      sensoresAtualizados(false),
      totalMovimentos(0),
      movimentosExploracao(0),
      movimentosRotaOtimizada(0) {
    
    subSensores = this->create_subscription<cg_interfaces::msg::RobotSensors>(
        "/culling_games/robot_sensors",
        10,
        std::bind(&ExploradorLabirinto::processarSensores, this, std::placeholders::_1)
    );
    
    clienteMovimento = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    clienteReset = this->create_client<cg_interfaces::srv::Reset>("/reset");
    
    std::cout << "\n╔════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   EXPLORADOR COM BACKTRACKING INICIADO            ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════╝\n" << std::endl;
}

void ExploradorLabirinto::executarExploracao() {
    std::cout << "╔════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     FASE 1: EXPLORANDO COM BACKTRACKING           ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════╝\n" << std::endl;
    
    std::cout << "⏳ Aguardando serviços..." << std::flush;
    if (!clienteMovimento->wait_for_service(std::chrono::seconds(5))) {
        std::cout << " ❌ ERRO\n";
        return;
    }
    std::cout << " ✅ OK\n" << std::endl;
    
    explorarLabirinto();
    
    if (!alvoEncontrado) {
        std::cout << "\n❌ ERRO: Alvo não encontrado durante exploração!\n" << std::endl;
        return;
    }
    
    std::cout << "\n╔════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║       FASE 2: CALCULANDO MELHOR ROTA              ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════╝\n" << std::endl;
    
    std::vector<PosicaoRobo> rotaOtimizada = calcularMelhorRota();
    
    if (rotaOtimizada.empty()) {
        std::cout << "❌ ERRO: Não foi possível calcular rota!\n" << std::endl;
        return;
    }
    
    std::cout << "✅ Rota otimizada encontrada!\n" << std::endl;
    std::cout << "┌────────────────────────────────────────────────────┐" << std::endl;
    std::cout << "│  COMPARAÇÃO DE ROTAS                               │" << std::endl;
    std::cout << "├────────────────────────────────────────────────────┤" << std::endl;
    std::cout << "│  Exploração inicial:  " << movimentosExploracao << " movimentos" 
              << std::string(24 - std::to_string(movimentosExploracao).length(), ' ') << "│" << std::endl;
    std::cout << "│  Rota otimizada:      " << (rotaOtimizada.size() - 1) << " movimentos"
              << std::string(24 - std::to_string(rotaOtimizada.size() - 1).length(), ' ') << "│" << std::endl;
    
    int economia = movimentosExploracao - (rotaOtimizada.size() - 1);
    float percentual = (100.0 * economia / movimentosExploracao);
    std::cout << "│  Economia:            " << economia << " movimentos (" 
              << static_cast<int>(percentual) << "%)"
              << std::string(20 - std::to_string(economia).length() - std::to_string(static_cast<int>(percentual)).length(), ' ') << "│" << std::endl;
    std::cout << "└────────────────────────────────────────────────────┘\n" << std::endl;
    
    std::cout << "╔════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║    FASE 3: EXECUTANDO ROTA OTIMIZADA              ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════╝\n" << std::endl;
    
    resetarJogo();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    executarRotaOtimizada(rotaOtimizada);
    
    mostrarEstatisticas();
}

void ExploradorLabirinto::explorarLabirinto() {
    rclcpp::Rate rate(10);
    
    std::cout << "📡 Aguardando primeira leitura dos sensores..." << std::flush;
    
    while (rclcpp::ok() && !sensoresAtualizados) {
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
    std::cout << " ✅\n" << std::endl;
    
    celulasVisitadas.insert(posicaoAtual);
    
    // 🔥 PILHA PARA BACKTRACKING
    std::stack<PosicaoRobo> pilhaCaminho;
    pilhaCaminho.push(posicaoAtual);
    
    int iteracoes = 0;
    const int MAX_ITERACOES = 10000;
    
    std::cout << "🤖 Iniciando exploração com backtracking...\n" << std::endl;
    
    while (rclcpp::ok() && !alvoEncontrado && iteracoes < MAX_ITERACOES) {
        sensoresAtualizados = false;
        
        int tentativas = 0;
        while (!sensoresAtualizados && tentativas < 20) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            tentativas++;
        }
        
        if (!sensoresAtualizados) {
            std::cout << "⚠️  Timeout aguardando sensores" << std::endl;
            break;
        }
        
        if (ultimosSensores) {
            atualizarMapa(ultimosSensores);
        }
        
        // 🔥 DECISÃO COM BACKTRACKING
        std::string proximaDirecao = decidirProximoMovimentoComBacktracking(pilhaCaminho);
        
        if (proximaDirecao.empty()) {
            std::cout << "\n⚠️  Exploração completa - alvo não encontrado!" << std::endl;
            std::cout << "   Células visitadas: " << celulasVisitadas.size() << std::endl;
            break;
        }
        
        auto offset = getOffset(proximaDirecao);
        int proxX = posicaoAtual.x + offset.first;
        int proxY = posicaoAtual.y + offset.second;
        
        // Verificar se é movimento de backtrack (voltar) ou exploração (avançar)
        bool ehBacktrack = false;
        if (pilhaCaminho.size() > 1) {
            std::stack<PosicaoRobo> copia = pilhaCaminho;
            copia.pop(); // Remove topo atual
            PosicaoRobo anterior = copia.top();
            if (proxX == anterior.x && proxY == anterior.y) {
                ehBacktrack = true;
            }
        }
        
        if (ehBacktrack) {
            std::cout << "🔙 Backtrack " << (iteracoes + 1) << ": " << proximaDirecao 
                      << " → (" << proxX << ", " << proxY << ")" << std::flush;
        } else {
            std::cout << "🔸 Movimento " << (iteracoes + 1) << ": " << proximaDirecao 
                      << " → (" << proxX << ", " << proxY << ")" << std::flush;
        }
        
        if (moverRobo(proximaDirecao)) {
            std::cout << " ✅" << std::endl;
            
            // 🔥 GERENCIAR PILHA
            if (ehBacktrack) {
                pilhaCaminho.pop(); // Remove posição anterior
            } else {
                pilhaCaminho.push(posicaoAtual); // Adiciona nova posição
            }
        } else {
            std::cout << " ❌" << std::endl;
            break;
        }
        
        iteracoes++;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    
    if (iteracoes >= MAX_ITERACOES) {
        std::cout << "\n⚠️  Timeout de exploração atingido!" << std::endl;
    }
    
    movimentosExploracao = totalMovimentos;
    std::cout << "\n✅ Exploração concluída: " << movimentosExploracao << " movimentos\n" << std::endl;
    
    if (alvoEncontrado) {
        std::cout << "🎯 Alvo encontrado em: (" << posicaoAlvo.x << ", " << posicaoAlvo.y << ")\n" << std::endl;
    }
    
    mostrarMapa();
}

// 🔥 NOVA FUNÇÃO: Decisão com backtracking
std::string ExploradorLabirinto::decidirProximoMovimentoComBacktracking(std::stack<PosicaoRobo>& pilhaCaminho) {
    // PASSO 1: Tentar células adjacentes não visitadas (exploração)
    std::vector<std::pair<int, int>> direcoes = {
        {0, 1},   // up
        {1, 0},   // right
        {0, -1},  // down
        {-1, 0}   // left
    };
    
    if (ultimosSensores) {
        std::cout << "   [Sensores: up=" << ultimosSensores->up 
                  << " down=" << ultimosSensores->down
                  << " left=" << ultimosSensores->left 
                  << " right=" << ultimosSensores->right << "]" << std::endl;
    }
    
    for (const auto& dir : direcoes) {
        int nx = posicaoAtual.x + dir.first;
        int ny = posicaoAtual.y + dir.second;
        PosicaoRobo proxima = {nx, ny};
        
        TipoCelula tipoCelula = getTipoCelula(nx, ny);
        
        if (celulasVisitadas.find(proxima) == celulasVisitadas.end() &&
            (tipoCelula == LIVRE || tipoCelula == ALVO)) {
            
            if (mapaDir.find(dir) != mapaDir.end()) {
                std::cout << "   → Explorando: " << mapaDir[dir] << " (célula: ";
                if (tipoCelula == LIVRE) std::cout << "livre";
                else if (tipoCelula == ALVO) std::cout << "alvo";
                std::cout << ")" << std::endl;
                return mapaDir[dir];
            }
        }
    }
    
    // PASSO 2: Não há células novas adjacentes - FAZER BACKTRACK
    if (pilhaCaminho.size() > 1) {
        std::stack<PosicaoRobo> copia = pilhaCaminho;
        copia.pop(); // Remove posição atual
        PosicaoRobo posicaoAnterior = copia.top();
        
        std::cout << "   → Fazendo backtrack para (" 
                  << posicaoAnterior.x << ", " << posicaoAnterior.y << ")" << std::endl;
        
        // Calcular direção para voltar
        std::pair<int, int> diff = {
            posicaoAnterior.x - posicaoAtual.x,
            posicaoAnterior.y - posicaoAtual.y
        };
        
        if (mapaDir.find(diff) != mapaDir.end()) {
            return mapaDir[diff];
        }
    }
    
    // PASSO 3: Pilha vazia - exploração completa sem encontrar alvo
    std::cout << "   ⚠️  Nenhuma célula para explorar e pilha vazia!" << std::endl;
    return "";
}

void ExploradorLabirinto::processarSensores(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
    ultimosSensores = msg;
    sensoresAtualizados = true;
}

void ExploradorLabirinto::atualizarMapa(const cg_interfaces::msg::RobotSensors::SharedPtr sensores) {
    std::map<std::string, std::pair<int, int>> offsets = {
        {"up", {0, 1}},
        {"down", {0, -1}},
        {"left", {-1, 0}},
        {"right", {1, 0}},
        {"up_left", {-1, 1}},
        {"up_right", {1, 1}},
        {"down_left", {-1, -1}},
        {"down_right", {1, -1}}
    };
    
    std::map<std::string, std::string> valores = {
        {"up", sensores->up},
        {"down", sensores->down},
        {"left", sensores->left},
        {"right", sensores->right},
        {"up_left", sensores->up_left},
        {"up_right", sensores->up_right},
        {"down_left", sensores->down_left},
        {"down_right", sensores->down_right}
    };

    std::set<std::string> direcoesCardeais = {"up", "down", "left", "right"};
    
    int celulasLivres = 0;
    int paredes = 0;
    
    for (const auto& [dir, offset] : offsets) {
        int nx = posicaoAtual.x + offset.first;
        int ny = posicaoAtual.y + offset.second;
        std::string valor = valores[dir];
        
        TipoCelula tipo = DESCONHECIDO;
        
        if (valor == "f") {
            tipo = LIVRE;
            celulasLivres++;
        }
        else if (valor == "b") {
            tipo = PAREDE;
            paredes++;
        }
        else if (valor == "t") {
            alvoEncontrado = true;
            bool direcaoCardinal = (direcoesCardeais.find(dir) != direcoesCardeais.end());
            tipo = ALVO;
            celulasLivres++;
            posicaoAlvo = {nx, ny};
            if (direcaoCardinal && alvoEncontrado) {
                std::cout << "\n\n🎯 ═══════════════════════════════════════════════════" << std::endl;
                std::cout << "   ALVO ENCONTRADO em (" << nx << ", " << ny << ")!" << std::endl;
                std::cout << "   ═══════════════════════════════════════════════════\n" << std::endl;
            }
        }
        
        mapaExplorado[{nx, ny}] = tipo;
        
        if (tipo == LIVRE || tipo == ALVO) {
            PosicaoRobo novaPosicao = {nx, ny};
            if (celulasVisitadas.find(novaPosicao) == celulasVisitadas.end()) {
                fronteira.push(novaPosicao);
            }
        }
    }
    
    mapaExplorado[{posicaoAtual.x, posicaoAtual.y}] = LIVRE;
    
    std::cout << "   [Mapa atualizado: " << celulasLivres << " livres, " 
              << paredes << " bloqueadas]" << std::endl;
}

// Manter a função original para compatibilidade
std::string ExploradorLabirinto::decidirProximoMovimento() {
    std::stack<PosicaoRobo> pilhaTemp;
    pilhaTemp.push(posicaoAtual);
    return decidirProximoMovimentoComBacktracking(pilhaTemp);
}

bool ExploradorLabirinto::moverRobo(const std::string& direcao) {
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direcao;
    
    movimentoEmAndamento = true;
    
    auto futuro = clienteMovimento->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), futuro, std::chrono::seconds(5)) 
        == rclcpp::FutureReturnCode::SUCCESS) {
        
        auto resposta = futuro.get();
        movimentoEmAndamento = false;
        
        if (resposta->success) {
            for (const auto& [offset, dir] : mapaDir) {
                if (dir == direcao) {
                    posicaoAtual.x += offset.first;
                    posicaoAtual.y += offset.second;
                    break;
                }
            }
            
            celulasVisitadas.insert(posicaoAtual);
            totalMovimentos++;
            return true;
        }
    }
    
    movimentoEmAndamento = false;
    return false;
}

std::pair<int, int> ExploradorLabirinto::getOffset(const std::string& direcao) {
    for (const auto& [offset, dir] : mapaDir) {
        if (dir == direcao) {
            return offset;
        }
    }
    return {0, 0};
}

std::vector<PosicaoRobo> ExploradorLabirinto::calcularMelhorRota() {
    return buscaEmLargura(posicaoInicial, posicaoAlvo);
}

std::vector<PosicaoRobo> ExploradorLabirinto::buscaEmLargura(PosicaoRobo inicio, PosicaoRobo alvo) {
    std::map<PosicaoRobo, PosicaoRobo> pai;
    std::queue<PosicaoRobo> fila;
    std::set<PosicaoRobo> visitados;
    
    fila.push(inicio);
    visitados.insert(inicio);
    pai[inicio] = inicio;
    
    while (!fila.empty()) {
        PosicaoRobo atual = fila.front();
        fila.pop();
        
        if (atual == alvo) {
            std::vector<PosicaoRobo> caminho;
            PosicaoRobo pos = alvo;
            
            while (!(pos == inicio)) {
                caminho.push_back(pos);
                pos = pai[pos];
            }
            caminho.push_back(inicio);
            
            std::reverse(caminho.begin(), caminho.end());
            return caminho;
        }
        
        std::vector<std::pair<int, int>> direcoes = {
            {0, 1}, {1, 0}, {0, -1}, {-1, 0}
        };
        
        for (const auto& dir : direcoes) {
            PosicaoRobo vizinho = {atual.x + dir.first, atual.y + dir.second};
            
            TipoCelula tipoCelula = getTipoCelula(vizinho.x, vizinho.y);
            
            if (visitados.find(vizinho) == visitados.end() &&
                (tipoCelula == LIVRE || tipoCelula == ALVO)) {
                
                visitados.insert(vizinho);
                pai[vizinho] = atual;
                fila.push(vizinho);
            }
        }
    }
    
    return {};
}

void ExploradorLabirinto::resetarJogo() {
    if (!clienteReset->wait_for_service(std::chrono::seconds(5))) {
        std::cout << "❌ Serviço /reset não disponível!" << std::endl;
        return;
    }
    
    auto request = std::make_shared<cg_interfaces::srv::Reset::Request>();
    request->is_random = false;
    
    std::cout << "🔄 Resetando jogo..." << std::flush;
    
    auto futuro = clienteReset->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), futuro, std::chrono::seconds(5))
        == rclcpp::FutureReturnCode::SUCCESS) {
        std::cout << " ✅\n" << std::endl;
        posicaoAtual = posicaoInicial;
        totalMovimentos = 0;
    } else {
        std::cout << " ❌\n" << std::endl;
    }
}

void ExploradorLabirinto::executarRotaOtimizada(const std::vector<PosicaoRobo>& rota) {
    std::cout << "🎯 Executando rota otimizada (" << (rota.size() - 1) << " passos)...\n" << std::endl;
    
    for (size_t i = 0; i < rota.size() - 1; i++) {
        std::string direcao = calcularDirecao(rota[i], rota[i + 1]);
        
        std::cout << "🔸 Passo " << (i + 1) << "/" << (rota.size() - 1) 
                  << ": " << direcao << " → (" << rota[i+1].x << ", " << rota[i+1].y << ")" << std::flush;
        
        if (moverRobo(direcao)) {
            std::cout << " ✅" << std::endl;
            movimentosRotaOtimizada++;
        } else {
            std::cout << " ❌ FALHA!" << std::endl;
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
    
    std::cout << "\n🏁 Rota otimizada concluída!\n" << std::endl;
}

std::string ExploradorLabirinto::calcularDirecao(const PosicaoRobo& atual, const PosicaoRobo& proximo) {
    std::pair<int, int> diff = {proximo.x - atual.x, proximo.y - atual.y};
    
    if (mapaDir.find(diff) != mapaDir.end()) {
        return mapaDir[diff];
    }
    
    return "";
}

TipoCelula ExploradorLabirinto::getTipoCelula(int x, int y) {
    auto it = mapaExplorado.find({x, y});
    if (it != mapaExplorado.end()) {
        return it->second;
    }
    return DESCONHECIDO;
}

void ExploradorLabirinto::mostrarMapa() {
    std::cout << "┌────────────────────────────────────────────────────┐" << std::endl;
    std::cout << "│              MAPA EXPLORADO                        │" << std::endl;
    std::cout << "└────────────────────────────────────────────────────┘\n" << std::endl;
    
    if (mapaExplorado.empty()) {
        std::cout << "  (vazio)\n" << std::endl;
        return;
    }
    
    int minX = INT_MAX, maxX = INT_MIN;
    int minY = INT_MAX, maxY = INT_MIN;
    
    for (const auto& [pos, _] : mapaExplorado) {
        minX = std::min(minX, pos.first);
        maxX = std::max(maxX, pos.first);
        minY = std::min(minY, pos.second);
        maxY = std::max(maxY, pos.second);
    }
    
    std::cout << "  Legenda: R = Robô | 🎯 = Alvo | · = Livre | █ = Parede\n" << std::endl;
    
    for (int y = maxY; y >= minY; y--) {
        std::cout << "  ";
        for (int x = minX; x <= maxX; x++) {
            if (x == posicaoAtual.x && y == posicaoAtual.y) {
                std::cout << "R ";
            } else if (x == posicaoAlvo.x && y == posicaoAlvo.y) {
                std::cout << "🎯";
            } else {
                TipoCelula tipo = getTipoCelula(x, y);
                switch (tipo) {
                    case LIVRE: std::cout << "· "; break;
                    case PAREDE: std::cout << "█ "; break;
                    case ALVO: std::cout << "A "; break;
                    default: std::cout << "? "; break;
                }
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void ExploradorLabirinto::mostrarEstatisticas() {
    std::cout << "\n╔════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              ESTATÍSTICAS FINAIS                  ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════╣" << std::endl;
    
    std::cout << "║  Células exploradas:        " << celulasVisitadas.size();
    std::cout << std::string(23 - std::to_string(celulasVisitadas.size()).length(), ' ') << "║" << std::endl;
    
    std::cout << "║  Movimentos exploração:     " << movimentosExploracao;
    std::cout << std::string(23 - std::to_string(movimentosExploracao).length(), ' ') << "║" << std::endl;
    
    std::cout << "║  Movimentos rota ótima:     " << movimentosRotaOtimizada;
    std::cout << std::string(23 - std::to_string(movimentosRotaOtimizada).length(), ' ') << "║" << std::endl;
    
    if (movimentosExploracao > 0) {
        int economia = movimentosExploracao - movimentosRotaOtimizada;
        float percentual = 100.0 * economia / movimentosExploracao;
        std::cout << "║  Economia:                  " << economia << " movimentos (" 
                  << static_cast<int>(percentual) << "%)";
        int espacos = 52 - 30 - std::to_string(economia).length() - std::to_string(static_cast<int>(percentual)).length() - 13;
        std::cout << std::string(espacos, ' ') << "║" << std::endl;
    }
    
    std::cout << "╚════════════════════════════════════════════════════╝\n" << std::endl;
}