#include "ControladorRobo.h"
#include "Grafo.h"
#include <iostream>

ControladorRobo::ControladorRobo() 
    : Node("controlador_robo"), 
      indiceAtual(0), 
      movimentoEmAndamento(false) {
    clienteMovimento = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    
    std::cout << "[ControladorRobo] Nó criado, aguardando serviço /move_command..." << std::endl;
}

void ControladorRobo::setCaminhoCoordenadas(const std::vector<std::pair<int, int>>& caminho) {
    caminhoCoordenadas = caminho;
    indiceAtual = 0;
    movimentoEmAndamento = false;
}

void ControladorRobo::setCaminhoIDs(const std::vector<int>& caminho, Grafo* grafo) {
    caminhoIDs = caminho;
    caminhoCoordenadas.clear();
    
    std::cout << "[Debug] Convertendo " << caminho.size() << " IDs para coordenadas..." << std::endl;
    
    for (int id : caminho) {
        if (grafo->idToCoord.find(id) != grafo->idToCoord.end()) {
            auto coord = grafo->idToCoord[id];
            caminhoCoordenadas.push_back(coord);
            std::cout << "[Debug] ID " << id << " -> (" << coord.first << ", " << coord.second << ")" << std::endl;
        } else {
            std::cout << "[ERRO] ID " << id << " não encontrado no mapa!" << std::endl;
        }
    }
    
    std::cout << "[Debug] Total de coordenadas no caminho: " << caminhoCoordenadas.size() << std::endl;
    
    indiceAtual = 0;
    movimentoEmAndamento = false;
}

bool ControladorRobo::moverProximoPasso() {
    if (caminhoCoordenadas.empty() || indiceAtual >= caminhoCoordenadas.size() - 1) {
        std::cout << "Fim do caminho atingido!" << std::endl;
        return false;
    }
    
    if (movimentoEmAndamento) {
        std::cout << "Movimento já em andamento, aguarde..." << std::endl;
        return false;
    }
    
    std::pair<int, int> atual = caminhoCoordenadas[indiceAtual];
    std::pair<int, int> proximo = caminhoCoordenadas[indiceAtual + 1];
    
    std::pair<int, int> diferenca = {
        proximo.first - atual.first,
        proximo.second - atual.second
    };
    
    std::cout << "[Debug] Posição atual: (" << atual.first << ", " << atual.second << ")" << std::endl;
    std::cout << "[Debug] Próxima posição: (" << proximo.first << ", " << proximo.second << ")" << std::endl;
    std::cout << "[Debug] Diferença: (" << diferenca.first << ", " << diferenca.second << ")" << std::endl;
    
    if (direcoes.find(diferenca) == direcoes.end()) {
        std::cout << "[ERRO] Direção inválida: (" << diferenca.first << ", " << diferenca.second << ")" << std::endl;
        return false;
    }
    
    std::string direcao = direcoes[diferenca];
    std::cout << "[Debug] Direção calculada: " << direcao << std::endl;
    
    executarMovimento(direcao);
    return true;
}

void ControladorRobo::executarCaminhoCompleto() {
    if (caminhoCoordenadas.size() <= 1) {
        std::cout << "Caminho muito curto ou vazio!" << std::endl;
        return;
    }
    
    std::cout << "Iniciando movimento pelo caminho completo..." << std::endl;
    std::cout << "Total de passos: " << caminhoCoordenadas.size() - 1 << std::endl;
    
    // Aguardar serviço estar disponível
    if (!clienteMovimento->wait_for_service(std::chrono::seconds(5))) {
        std::cout << "[ERRO] Serviço /move_command não está disponível!" << std::endl;
        return;
    }
    
    std::cout << "[INFO] Serviço /move_command disponível. Iniciando movimento..." << std::endl;
    moverProximoPasso();
}

bool ControladorRobo::chegouAoAlvo() {
    return indiceAtual >= caminhoCoordenadas.size() - 1;
}

std::pair<int, int> ControladorRobo::getPosicaoAtual() {
    if (caminhoCoordenadas.empty()) return {-1, -1};
    return caminhoCoordenadas[indiceAtual];
}

std::pair<int, int> ControladorRobo::getProximaPosicao() {
    if (caminhoCoordenadas.empty() || indiceAtual >= caminhoCoordenadas.size() - 1) 
        return {-1, -1};
    return caminhoCoordenadas[indiceAtual + 1];
}

void ControladorRobo::executarMovimento(const std::string& direcao) {
    if (!clienteMovimento->wait_for_service(std::chrono::seconds(1))) {
        std::cout << "[ERRO] Serviço de movimento não disponível!" << std::endl;
        movimentoEmAndamento = false;
        return;
    }
    
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direcao;
    
    std::cout << "[INFO] Enviando comando de movimento: " << direcao << std::endl;
    
    movimentoEmAndamento = true;
    
    auto futuro = clienteMovimento->async_send_request(request,
        [this](rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future) {
            this->tratarRespostaMovimento(future);
        });
}

void ControladorRobo::tratarRespostaMovimento(rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future) {
    try {
        auto resposta = future.get();
        movimentoEmAndamento = false;
        
        std::cout << "[Debug] Resposta recebida - success: " << (resposta->success ? "true" : "false") << std::endl;
        
        if (resposta->success) {
            std::cout << "[SUCCESS] Movimento realizado com sucesso! ";
            indiceAtual++;
            
            std::pair<int, int> posAtual = getPosicaoAtual();
            std::cout << "Posição atual: (" << posAtual.first << ", " << posAtual.second << ")" << std::endl;
            std::cout << "Progresso: " << indiceAtual << "/" << (caminhoCoordenadas.size() - 1) << std::endl;
            
            if (!chegouAoAlvo()) {
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                moverProximoPasso();
            } else {
                std::cout << "\n🎉 ALVO ALCANÇADO! 🎉\n" << std::endl;
            }
        } else {
            std::cout << "[FALHA] Movimento falhou! ";
            
            // Debug: mostrar posição que tentou mover
            if (indiceAtual < caminhoCoordenadas.size() - 1) {
                auto atual = caminhoCoordenadas[indiceAtual];
                auto proximo = caminhoCoordenadas[indiceAtual + 1];
                std::cout << "Tentativa: (" << atual.first << ", " << atual.second << ") -> ("
                          << proximo.first << ", " << proximo.second << ")" << std::endl;
            }
            
            // Limitar tentativas para evitar loop infinito
            static int tentativasFalhadas = 0;
            tentativasFalhadas++;
            
            if (tentativasFalhadas > 5) {
                std::cout << "\n[ERRO CRÍTICO] Muitas falhas consecutivas. Abortando..." << std::endl;
                tentativasFalhadas = 0;
                return;
            }
            
            std::cout << "Tentando novamente em 1 segundo..." << std::endl;
            rclcpp::sleep_for(std::chrono::seconds(1));
            moverProximoPasso();
        }
    } catch (const std::exception& e) {
        std::cout << "[EXCEÇÃO] Erro ao processar resposta: " << e.what() << std::endl;
        movimentoEmAndamento = false;
    }
}