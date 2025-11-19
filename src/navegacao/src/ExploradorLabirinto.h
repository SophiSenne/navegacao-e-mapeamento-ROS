#ifndef EXPLORADOR_LABIRINTO_H
#define EXPLORADOR_LABIRINTO_H

#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/msg/robot_sensors.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/srv/reset.hpp>
#include <map>
#include <vector>
#include <queue>
#include <set>
#include <memory>
#include <stack>

enum TipoCelula {
    DESCONHECIDO = -1,
    LIVRE = 0,
    PAREDE = 1,
    ALVO = 2
};

struct PosicaoRobo {
    int x, y;
    
    bool operator<(const PosicaoRobo& outro) const {
        if (x != outro.x) return x < outro.x;
        return y < outro.y;
    }
    
    bool operator==(const PosicaoRobo& outro) const {
        return x == outro.x && y == outro.y;
    }
};

class ExploradorLabirinto : public rclcpp::Node {
public:
    ExploradorLabirinto();
    
    // Função principal que executa todo o processo
    void executarExploracao();

private:
    // === Fase 1: Exploração ===
    void explorarLabirinto();
    void processarSensores(const cg_interfaces::msg::RobotSensors::SharedPtr msg);
    void atualizarMapa(const cg_interfaces::msg::RobotSensors::SharedPtr sensores);
    std::string decidirProximoMovimento();
    bool moverRobo(const std::string& direcao);

    std::string decidirProximoMovimentoComBacktracking(std::stack<PosicaoRobo>& pilhaCaminho);
    
    // === Fase 2: Cálculo da melhor rota ===
    std::vector<PosicaoRobo> calcularMelhorRota();
    std::vector<PosicaoRobo> buscaEmLargura(PosicaoRobo inicio, PosicaoRobo alvo);
    
    // === Fase 3: Execução otimizada ===
    void resetarJogo();
    void executarRotaOtimizada(const std::vector<PosicaoRobo>& rota);
    std::string calcularDirecao(const PosicaoRobo& atual, const PosicaoRobo& proximo);
    
    // Utilitários
    std::vector<PosicaoRobo> getVizinhosLivres(const PosicaoRobo& pos);
    TipoCelula getTipoCelula(int x, int y);
    void mostrarEstatisticas();
    void mostrarMapa();
    std::pair<int, int> getOffset(const std::string& direcao);
    
    // ROS2
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr subSensores;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr clienteMovimento;
    rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr clienteReset;
    
    // Estado do robô
    PosicaoRobo posicaoAtual;
    PosicaoRobo posicaoInicial;
    PosicaoRobo posicaoAlvo;
    
    // Mapa descoberto
    std::map<std::pair<int, int>, TipoCelula> mapaExplorado;
    std::set<PosicaoRobo> celulasVisitadas;
    std::queue<PosicaoRobo> fronteira; // Células a explorar
    
    // Controle de estado
    bool alvoEncontrado;
    bool movimentoEmAndamento;
    bool sensoresAtualizados;
    cg_interfaces::msg::RobotSensors::SharedPtr ultimosSensores;
    
    // Estatísticas
    int totalMovimentos;
    int movimentosExploracao;
    int movimentosRotaOtimizada;
    
    // Mapeamento de direções
    std::map<std::pair<int, int>, std::string> mapaDir = {
        {{0, 1}, "up"},
        {{0, -1}, "down"},
        {{1, 0}, "right"},
        {{-1, 0}, "left"},
        {{1, 1}, "up_right"},
        {{-1, 1}, "up_left"},
        {{1, -1}, "down_right"},
        {{-1, -1}, "down_left"}
    };
};

#endif // EXPLORADOR_LABIRINTO_H