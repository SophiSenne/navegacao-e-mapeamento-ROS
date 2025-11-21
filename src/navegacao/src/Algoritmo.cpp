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
    
    std::cout << "Caminho encontrado" << std::endl;
}

void Algoritmo::executarBuscaEMovimento(std::shared_ptr<ControladorRobo> controlador) {
    int idInicio = grafo->coordToId[inicio];
    int idAlvo = grafo->coordToId[alvo];
    int vert = grafo->listaAdj.size();
    
    std::cout << "ID início: " << idInicio << " (coord: " 
              << inicio.first << ", " << inicio.second << ")" << std::endl;
    std::cout << "ID alvo: " << idAlvo << " (coord: " 
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
    
    std::vector<int> caminho;
    for (int v = idAlvo; v != -1; v = pai[v])
        caminho.push_back(v);
    reverse(caminho.begin(), caminho.end());
    
    std::cout << "\nCaminho encontrado com " << caminho.size() << " passos" << std::endl;
    
    if (caminho.size() > 1) {
        controlador->setCaminhoIDs(caminho, grafo);
        
        std::cout << std::endl;
        std::cout << "Iniciando execução do caminho" << std::endl;
        controlador->executarCaminhoCompleto();
        
        rclcpp::Rate rate(20);
        int iteracoes = 0;
        
        while (rclcpp::ok() && !controlador->chegouAoAlvo()) {
            rclcpp::spin_some(controlador);
            rate.sleep();
            
            iteracoes++;
            
            if (iteracoes > 3000) {
                std::cout << "\nTimeout atingido. Abortando..." << std::endl;
                break;
            }
        }
        
        if (controlador->chegouAoAlvo()) {
            std::cout << "\n✅ Navegação concluída com sucesso!" << std::endl;
        } else {
            std::cout << "\n⚠️ Navegação interrompida." << std::endl;
        }
        
    } else {
        std::cout << "[ERRO] Caminho não encontrado ou inválido (tamanho: " 
                  << caminho.size() << ")" << std::endl;
    }
}