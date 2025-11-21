#include "Grafo.h"
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

Grafo::Grafo(int argc, char** argv) {
    lerMapaROS();
    construirArestas();
}

void Grafo::lerCSV() {
    std::ifstream arquivo(nomeArquivo);
    if (!arquivo.is_open()) {
        std::cerr << "Erro ao abrir arquivo " << nomeArquivo << std::endl;
        exit(1);
    }

    std::string linha;
    int linhaAtual = 0;

    while (getline(arquivo, linha)) {
        std::vector<char> linhaGrid;
        std::stringstream ss(linha);
        std::string celula;
        int colunaAtual = 0;

        while (getline(ss, celula, ',')) {
            char valor = celula[0];
            linhaGrid.push_back(valor);

            if (valor != 'b') {
                coordToId[{linhaAtual, colunaAtual}] = idContador;
                idToCoord[idContador] = {linhaAtual, colunaAtual};  // Adiciona mapa reverso
                idContador++;
            }
            colunaAtual++;
        }

        mapa.push_back(linhaGrid);
        numColunas = colunaAtual;
        linhaAtual++;
    }

    numLinhas = linhaAtual;
    arquivo.close();

    std::cout << "Leitura concluída: " << numLinhas << " linhas, "
              << numColunas << " colunas." << std::endl;
    std::cout << "Encontrados " << idContador << " vértices." << std::endl;
}

void Grafo::lerMapaROS() {
    auto node = rclcpp::Node::make_shared("grafo_map_reader");
    
    auto client = node->create_client<cg_interfaces::srv::GetMap>("/get_map");
    
    std::cout << "Aguardando serviço /get_map" << std::endl;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            std::cerr << "Interrompido enquanto aguardava o serviço." << std::endl;
            exit(1);
        }
        std::cout << "Serviço não disponível, aguardando" << std::endl;
    }
    
    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
    std::cout << "Chamando serviço /get_map..." << std::endl;
    auto future = client->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node, future) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        
        auto response = future.get();
        if (response->occupancy_grid_shape.size() != 2) {
            std::cerr << "Shape inválido do mapa!" << std::endl;
            exit(1);
        }
        
        numLinhas = response->occupancy_grid_shape[0];
        numColunas = response->occupancy_grid_shape[1];
        
        std::cout << "Mapa recebido: " << numLinhas << " linhas, " 
                  << numColunas << " colunas." << std::endl;
        
        int idx = 0;
        for (int r = 0; r < numLinhas; r++) {
            std::vector<char> linhaGrid;
            for (int c = 0; c < numColunas; c++) {
                char valor = response->occupancy_grid_flattened[idx][0];
                linhaGrid.push_back(valor);
                
                if (valor != 'b') {
                    coordToId[{r, c}] = idContador;
                    idToCoord[idContador] = {r, c}; 
                    idContador++;
                    
                    if (valor == 'r')
                        coorRobo = {r, c};
                    if (valor == 't')
                        coorAlvo = {r, c};
                }
                idx++;
            }
            mapa.push_back(linhaGrid);
        }
        
        std::cout << "Encontrados " << idContador << " vértices." << std::endl;
        
    } else {
        std::cerr << "Falha ao chamar o serviço /get_map" << std::endl;
        exit(1);
    }
}

void Grafo::construirArestas() {
    std::cout << "Construindo arestas" << std::endl;

    listaAdj.resize(idContador);

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    for (int r = 0; r < numLinhas; r++) {
        for (int c = 0; c < numColunas; c++) {
            if (mapa[r][c] == 'b') continue;

            int idAtual = coordToId[{r, c}];

            for (int k = 0; k < 4; k++) {
                int nr = r + dr[k];
                int nc = c + dc[k];

                if (nr < 0 || nr >= numLinhas || nc < 0 || nc >= numColunas)
                    continue;

                if (mapa[nr][nc] != 'b') {
                    int idViz = coordToId[{nr, nc}];
                    listaAdj[idAtual].push_back(idViz);
                }
            }
        }
    }

    std::cout << "Arestas construídas com sucesso" << std::endl;
}

void Grafo::imprimirGrafo() {
    std::cout << "\nLista de adjacência:\n";
    for (size_t id = 0; id < listaAdj.size(); id++) {
        std::cout << id << " -> ";
        for (int viz : listaAdj[id]) 
            std::cout << viz << " ";
        std::cout << std::endl;
    }
}