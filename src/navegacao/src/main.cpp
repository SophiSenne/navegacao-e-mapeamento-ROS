#include "Grafo.h"
#include "Algoritmo.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <sys/wait.h>

// Variável global para controlar o processo do maze
pid_t maze_pid = -1;

void limparTela() {
    std::cout << "\033[2J\033[1;1H";
}

void exibirBanner() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║          ROS2 MAZE NAVIGATOR - Sistema de Mapas         ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";
}

void exibirMenu() {
    std::cout << "┌──────────────────────────────────────────────────────────┐\n";
    std::cout << "│  Selecione uma opção:                                    │\n";
    std::cout << "├──────────────────────────────────────────────────────────┤\n";
    std::cout << "│  1. Mapear novo ambiente                                 │\n";
    std::cout << "│  2. Usar mapa pronto (executar navegação)                │\n";
    std::cout << "│  3. Sair                                                  │\n";
    std::cout << "└──────────────────────────────────────────────────────────┘\n";
    std::cout << "\nOpção: ";
}

bool iniciarMazeSimulador() {
    std::cout << "\n[INFO] Iniciando simulador maze...\n";
    
    maze_pid = fork();
    
    if (maze_pid == 0) {
        // Processo filho - executa o maze
        execlp("ros2", "ros2", "run", "cg", "maze", nullptr);
        // Se chegar aqui, houve erro
        std::cerr << "[ERRO] Falha ao executar ros2 run cg maze\n";
        exit(1);
    } else if (maze_pid > 0) {
        // Processo pai
        std::cout << "[INFO] Simulador iniciado (PID: " << maze_pid << ")\n";
        std::cout << "[INFO] Aguardando inicialização...\n";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        return true;
    } else {
        std::cerr << "[ERRO] Falha ao criar processo\n";
        return false;
    }
}

void pararMazeSimulador() {
    if (maze_pid > 0) {
        std::cout << "\n[INFO] Encerrando simulador...\n";
        kill(maze_pid, SIGTERM);
        waitpid(maze_pid, nullptr, 0);
        maze_pid = -1;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void modoMapear(int argc, char** argv) {
    limparTela();
    std::cout << "\n╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║              MODO MAPEAMENTO ATIVADO                     ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n\n";
    
    if (!iniciarMazeSimulador()) {
        std::cout << "\n[ERRO] Não foi possível iniciar o simulador.\n";
        std::cout << "Pressione ENTER para voltar ao menu...";
        std::cin.ignore();
        std::cin.get();
        return;
    }
    
    std::cout << "\n[INFO] Iniciando processo de mapeamento...\n";
    std::cout << "[INFO] Execute seus comandos de mapeamento em outro terminal\n";
    std::cout << "[INFO] Exemplo: ros2 run slam_toolbox async_slam_toolbox\n\n";
    
    std::cout << "┌──────────────────────────────────────────────────────────┐\n";
    std::cout << "│  Instruções:                                             │\n";
    std::cout << "│  - Use teleop para controlar o robô                      │\n";
    std::cout << "│  - Explore todo o ambiente                               │\n";
    std::cout << "│  - Salve o mapa quando finalizar                         │\n";
    std::cout << "└──────────────────────────────────────────────────────────┘\n";
    
    std::cout << "\nPressione ENTER quando terminar o mapeamento...";
    std::cin.ignore();
    std::cin.get();
    
    pararMazeSimulador();
    
    std::cout << "\n[INFO] Modo mapeamento finalizado.\n";
    std::cout << "Pressione ENTER para voltar ao menu...";
    std::cin.get();
}

void modoNavegacao(int argc, char** argv) {
    limparTela();
    std::cout << "\n╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║           MODO NAVEGAÇÃO - MAPA PRONTO                   ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n\n";
    
    if (!iniciarMazeSimulador()) {
        std::cout << "\n[ERRO] Não foi possível iniciar o simulador.\n";
        std::cout << "Pressione ENTER para voltar ao menu...";
        std::cin.ignore();
        std::cin.get();
        return;
    }
    
    std::cout << "[INFO] Inicializando ROS2...\n";
    rclcpp::init(argc, argv);
    
    try {
        std::cout << "[INFO] Criando grafo a partir do mapa...\n";
        Grafo grafo(argc, argv);
        
        std::cout << "\n┌──────────────────────────────────────────────────────────┐\n";
        std::cout << "│  INFORMAÇÕES DO GRAFO:                                   │\n";
        std::cout << "└──────────────────────────────────────────────────────────┘\n\n";
        
        // grafo.imprimirGrafo();
        
        std::cout << "\n[INFO] Grafo construído com sucesso!\n";
        
        std::cout << "\n[INFO] Executando busca em largura...\n";

        Algoritmo algoritmo(&grafo, grafo.coorRobo, grafo.coorAlvo);
        algoritmo.buscaEmLargura();
        
        std::cout << "\n[INFO] Navegação concluída!\n";
        std::cout << "\nPressione ENTER para finalizar e voltar ao menu...";
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
    std::cout << "\n\n[INFO] Sinal de interrupção recebido. Encerrando...\n";
    pararMazeSimulador();
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char** argv) {
    // Configurar tratamento de sinais
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
            std::cout << "\n[ERRO] Opção inválida! Pressione ENTER para continuar...";
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
                std::cout << "\n[INFO] Encerrando sistema...\n";
                std::cout << "Até logo! 👋\n\n";
                pararMazeSimulador();
                return 0;
                
            default:
                std::cout << "\n[ERRO] Opção inválida! Pressione ENTER para continuar...";
                std::cin.ignore();
                std::cin.get();
                break;
        }
    }
    
    return 0;
}