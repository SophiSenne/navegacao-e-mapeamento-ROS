#include "Grafo.h"
#include "Algoritmo.h"
#include "ControladorRobo.h"
#include "ExploradorLabirinto.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <sys/wait.h>

pid_t maze_pid = -1;

void limparTela() {
    std::cout << "\033[2J\033[1;1H";
}

void exibirBanner() {
    std::cout << std::endl;
    std::cout << "╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                    ROS2 MAZE NAVIGATOR                   ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;
}

void exibirMenu() {
    std::cout << "┌──────────────────────────────────────────────────────────┐" << std::endl;
    std::cout << "│  Selecione uma opção:                                    │" << std::endl;
    std::cout << "├──────────────────────────────────────────────────────────┤" << std::endl;
    std::cout << "│  1. Mapear novo ambiente                                 │" << std::endl;
    std::cout << "│  2. Executar navegação                                   │" << std::endl;
    std::cout << "│  3. Sair                                                 │" << std::endl;
    std::cout << "└──────────────────────────────────────────────────────────┘" << std::endl;
    std::cout << std::endl << "Opção: ";
}

bool iniciarMazeSimulador() {
    int opcao;
    std::string nome_mapa;

    std::cout << "┌──────────────────────────────────────────────────────────┐\n";
    std::cout << "│  Selecione uma opção:                                    │\n";
    std::cout << "├──────────────────────────────────────────────────────────┤\n";
    std::cout << "│  1. Carregar mapa aleatório (padrão)                     │\n";
    std::cout << "│  2. Carregar mapa específico                             │\n";
    std::cout << "│  3. Gerar novo labirinto                                 │\n";
    std::cout << "│  4. Sair                                                 │\n";
    std::cout << "└──────────────────────────────────────────────────────────┘\n";
    std::cout << "\nOpção: ";
    std::cin >> opcao;

    std::vector<const char*> args = {"ros2", "run", "cg", "maze"};

    switch (opcao) {
        case 1:
            break;

        case 2:
            std::cout << "Digite o nome do mapa (ex: test.csv): ";
            std::cin >> nome_mapa;
            args.push_back("--");
            args.push_back("--map");
            args.push_back(nome_mapa.c_str());
            break;

        case 3:
            args.push_back("--");
            args.push_back("--generate");
            break;

        case 4:
            return false;

        default:
            std::cerr << "Opção inválida.\n";
            return false;
    }

    std::cout << std::endl;
    std::cout << "Iniciando simulador\n";

    maze_pid = fork();
    if (maze_pid == 0) {
        args.push_back(nullptr);

        execvp("ros2", (char* const*)args.data());
        std::cerr << "[ERRO] Falha ao executar ros2 run cg maze\n";
        exit(1);

    } else if (maze_pid > 0) {
        std::cout << std::endl;
        std::cout << "Simulador iniciado (PID: " << maze_pid << ")\n";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        return true;

    } else {
        std::cerr << "[ERRO] Falha ao criar processo\n";
        return false;
    }
}

void pararMazeSimulador() {
    if (maze_pid > 0) {
        std::cout << std::endl;
        std::cout << "Encerrando simulador\n";
        kill(maze_pid, SIGTERM);
        rclcpp::shutdown();
        waitpid(maze_pid, nullptr, 0);
        maze_pid = -1;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void modoMapear(int argc, char** argv) {
    limparTela();
    std::cout << std::endl;
    std::cout << "╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                     MODO MAPEAMENTO                      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    
    if (!iniciarMazeSimulador()) {
        std::cout << "\n[ERRO] Não foi possível iniciar o simulador.\n";
        std::cout << "Pressione ENTER para voltar ao menu";
        std::cin.ignore();
        std::cin.get();
        return;
    }

    rclcpp::init(argc, argv);
    
    auto explorador = std::make_shared<ExploradorLabirinto>();
    explorador->executarExploracao();
    
    rclcpp::shutdown();
    pararMazeSimulador();
    
    std::cout << "\n[INFO] Modo mapeamento finalizado.\n";
    std::cout << "Pressione ENTER para voltar ao menu";
    std::cin.get();
}

void modoNavegacao(int argc, char** argv) {
    limparTela();
    std::cout << std::endl;
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║                      MODO NAVEGAÇÃO                      ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n\n";
    
    if (!iniciarMazeSimulador()) {
        std::cout << "\n[ERRO] Não foi possível iniciar o simulador.\n";
        std::cout << "Pressione ENTER para voltar ao menu";
        std::cin.ignore();
        std::cin.get();
        return;
    }
    
    rclcpp::init(argc, argv);
    
    try {
        std::cout << std::endl;
        std::cout << "Criando grafo a partir do mapa\n";
        Grafo grafo(argc, argv);
             
        std::cout << "\nGrafo construído com sucesso!\n";
        
        std::cout << "\nExecutando busca em largura\n";

        Algoritmo algoritmo(&grafo, grafo.coorRobo, grafo.coorAlvo);
        auto controlador = std::make_shared<ControladorRobo>();
        algoritmo.buscaEmLargura();
        algoritmo.executarBuscaEMovimento(controlador);
        
        std::cout << "\nPressione ENTER para finalizar e voltar ao menu";
        std::cin.ignore();
        std::cin.get();

    } catch (const std::exception& e) {
        std::cerr << "\n[ERRO] Exceção capturada: " << e.what() << "\n";
        std::cout << "Pressione ENTER para voltar ao menu...";
        std::cin.ignore();
        std::cin.get();
    }
    
    rclcpp::shutdown();
    pararMazeSimulador();
}

void tratarSinalInterrupcao(int sig) {
    std::cout << "\n\n[INFO] Sinal de interrupção recebido. Encerrando\n";
    pararMazeSimulador();
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char** argv) {
    signal(SIGINT, tratarSinalInterrupcao);
    signal(SIGTERM, tratarSinalInterrupcao);
    
    int opcao = 0;
    
    while (true) {
        limparTela();
        exibirBanner();
        exibirMenu();
        
        std::cin >> opcao;
        
        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "\nOpção inválida! Pressione ENTER para continuar";
            std::cin.get();
            continue;
        }
        
        switch (opcao) {
            case 1:
                modoMapear(argc, argv);
                break;
                
            case 2:
                modoNavegacao(argc, argv);
                break;
                
            case 3:
                limparTela();
                std::cout << "\nEncerrando sistema\n";
                std::cout << "Até logo! 👋\n\n";
                rclcpp::shutdown();
                pararMazeSimulador();
                return 0;
                
            default:
                std::cout << "\nOpção inválida! Pressione ENTER para continuar";
                std::cin.ignore();
                std::cin.get();
                break;
        }
    }
    
    return 0;
}