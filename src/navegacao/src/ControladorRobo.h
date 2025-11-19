#ifndef CONTROLADORROBO_H
#define CONTROLADORROBO_H

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <map>
#include <utility>
#include <string>
#include <cg_interfaces/srv/move_cmd.hpp>

// Forward declaration
class Grafo;

using namespace std::chrono_literals;

class ControladorRobo : public rclcpp::Node {
private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr clienteMovimento;
    std::vector<std::pair<int, int>> caminhoCoordenadas;
    std::vector<int> caminhoIDs;
    size_t indiceAtual;
    bool movimentoEmAndamento;
    
    // Mapeamento de direções
    std::map<std::pair<int, int>, std::string> direcoes = {
        {{1, 0}, "down"},
        {{-1, 0}, "up"}, 
        {{0, 1}, "right"},
        {{0, -1}, "left"}
    };

public:
    ControladorRobo();
    
    void setCaminhoCoordenadas(const std::vector<std::pair<int, int>>& caminho);
    void setCaminhoIDs(const std::vector<int>& caminho, Grafo* grafo);
    
    bool moverProximoPasso();
    void executarCaminhoCompleto();
    bool chegouAoAlvo();
    
    std::pair<int, int> getPosicaoAtual();
    std::pair<int, int> getProximaPosicao();

private:
    void executarMovimento(const std::string& direcao);
    void tratarRespostaMovimento(rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future);
};

#endif