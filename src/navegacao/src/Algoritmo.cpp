#include "Algoritmo.h"
#include "ControladorRobo.h"
#include "Grafo.h"
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>

Algoritmo::Algoritmo(Grafo* g, std::pair<int, int> i, std::pair<int, int> a) 
    : grafo(g), inicio(i), alvo(a) {
}

void Algoritmo::buscaEmLargura() {
    int idInicio = grafo->coordToId[inicio];
    int idAlvo = grafo->coordToId[alvo];
    int vert = grafo->listaAdj.size();
    
    std::vector<bool> visitado(vert, false);
    std::vector<int> pai(vert, -1);
    std::queue<int> fila;
    
    fila.push(idInicio);
    visitado[idInicio] = true;
    
    while (!fila.empty()) {
        int atual = fila.front();
        fila.pop();
        
        if (atual == idAlvo)
            break;
        
        for (int viz : grafo->listaAdj[atual]) {
            if (!visitado[viz]) {
                visitado[viz] = true;
                pai[viz] = atual;
                fila.push(viz);
            }
        }
    }
    
    // Reconstruir caminho
    std::vector<int> caminho;
    for (int v = idAlvo; v != -1; v = pai[v])
        caminho.push_back(v);
    reverse(caminho.begin(), caminho.end());
    
    std::cout << "Caminho encontrado (IDs): ";
    for (int id : caminho) 
        std::cout << id << " ";
    std::cout << std::endl;
}

void Algoritmo::executarBuscaEMovimento(std::shared_ptr<ControladorRobo> controlador) {
    std::cout << "\n[Algoritmo] Iniciando busca em largura..." << std::endl;
    
    // Executar busca
    int idInicio = grafo->coordToId[inicio];
    int idAlvo = grafo->coordToId[alvo];
    int vert = grafo->listaAdj.size();
    
    std::cout << "[Algoritmo] ID início: " << idInicio << " (coord: " 
              << inicio.first << ", " << inicio.second << ")" << std::endl;
    std::cout << "[Algoritmo] ID alvo: " << idAlvo << " (coord: " 
              << alvo.first << ", " << alvo.second << ")" << std::endl;
    
    std::vector<bool> visitado(vert, false);
    std::vector<int> pai(vert, -1);
    std::queue<int> fila;
    
    fila.push(idInicio);
    visitado[idInicio] = true;
    
    while (!fila.empty()) {
        int atual = fila.front();
        fila.pop();
        
        if (atual == idAlvo)
            break;
        
        for (int viz : grafo->listaAdj[atual]) {
            if (!visitado[viz]) {
                visitado[viz] = true;
                pai[viz] = atual;
                fila.push(viz);
            }
        }
    }
    
    // Reconstruir caminho
    std::vector<int> caminho;
    for (int v = idAlvo; v != -1; v = pai[v])
        caminho.push_back(v);
    reverse(caminho.begin(), caminho.end());
    
    std::cout << "\n[Algoritmo] Caminho encontrado com " << caminho.size() << " passos:" << std::endl;
    std::cout << "IDs: ";
    for (size_t i = 0; i < std::min(caminho.size(), size_t(10)); i++) 
        std::cout << caminho[i] << " ";
    if (caminho.size() > 10)
        std::cout << "... (" << (caminho.size() - 10) << " mais)";
    std::cout << std::endl;
    
    // Executar movimento
    if (caminho.size() > 1) {
        std::cout << "\n[Algoritmo] Configurando caminho no controlador..." << std::endl;
        controlador->setCaminhoIDs(caminho, grafo);
        
        std::cout << "[Algoritmo] Iniciando execução do caminho..." << std::endl;
        controlador->executarCaminhoCompleto();
        
        // Manter o nó ativo enquanto o robô se move
        std::cout << "[Algoritmo] Aguardando conclusão do movimento..." << std::endl;
        rclcpp::Rate rate(10);  // 10 Hz
        int iteracoes = 0;
        
        while (rclcpp::ok() && !controlador->chegouAoAlvo()) {
            rclcpp::spin_some(controlador);
            rate.sleep();
            
            iteracoes++;
            if (iteracoes % 50 == 0) {  // A cada 5 segundos
                std::cout << "[Algoritmo] Ainda em movimento... (iteração " << iteracoes << ")" << std::endl;
            }
            
            // Timeout de segurança - 5 minutos (3000 iterações a 10Hz)
            if (iteracoes > 3000) {
                std::cout << "\n[AVISO] Timeout atingido. Abortando..." << std::endl;
                break;
            }
        }
        
        if (controlador->chegouAoAlvo()) {
            std::cout << "\n✅ [Algoritmo] Navegação concluída com sucesso!" << std::endl;
        } else {
            std::cout << "\n⚠️ [Algoritmo] Navegação interrompida." << std::endl;
        }
        
    } else {
        std::cout << "[ERRO] Caminho não encontrado ou inválido (tamanho: " 
                  << caminho.size() << ")" << std::endl;
    }
}