#include "Grafo.h"

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"

#include <bits/stdc++.h>
using namespace std;

Grafo::Grafo(int argc, char** argv) {
    lerMapaROS();
    construirArestas();
}

void Grafo::lerCSV() {
    ifstream arquivo(nomeArquivo);
    if (!arquivo.is_open()) {
        cerr << "Erro ao abrir arquivo " << nomeArquivo << endl;
        exit(1);
    }

    string linha;
    int linhaAtual = 0;

    while (getline(arquivo, linha)) {
        vector<char> linhaGrid;
        stringstream ss(linha);
        string celula;
        int colunaAtual = 0;

        while (getline(ss, celula, ',')) {
            char valor = celula[0];
            linhaGrid.push_back(valor);

            if (valor != 'b') {
                coordToId[{linhaAtual, colunaAtual}] = idContador;
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

    cout << "Leitura concluída: " << numLinhas << " linhas, "
            << numColunas << " colunas." << endl;
    cout << "Encontrados " << idContador << " vértices." << endl;
}

void Grafo::lerMapaROS() {
    auto node = rclcpp::Node::make_shared("grafo_map_reader");
    
    auto client = node->create_client<cg_interfaces::srv::GetMap>("/get_map");
    
    cout << "Aguardando serviço /get_map" << endl;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            cerr << "Interrompido enquanto aguardava o serviço." << endl;
            exit(1);
        }
        cout << "Serviço não disponível, aguardando" << endl;
    }
    
    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
    cout << "Chamando serviço /get_map..." << endl;
    auto future = client->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node, future) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        
        auto response = future.get();
        if (response->occupancy_grid_shape.size() != 2) {
            cerr << "Shape inválido do mapa!" << endl;
            exit(1);
        }
        
        numLinhas = response->occupancy_grid_shape[0];
        numColunas = response->occupancy_grid_shape[1];
        
        cout << "Mapa recebido: " << numLinhas << " linhas, " 
             << numColunas << " colunas." << endl;
        
        int idx = 0;
        for (int r = 0; r < numLinhas; r++) {
            vector<char> linhaGrid;
            for (int c = 0; c < numColunas; c++) {
                char valor = response->occupancy_grid_flattened[idx][0];
                linhaGrid.push_back(valor);
                
                if (valor != 'b') {
                    coordToId[{r, c}] = idContador;
                    idContador++;
                    if(valor == 'r')
                        coorRobo = {r, c};
                    if(valor == 't')
                        coorAlvo = {r, c};
                }
                idx++;
            }
            mapa.push_back(linhaGrid);
        }
        
        cout << "Encontrados " << idContador << " vértices." << endl;
        
    } else {
        cerr << "Falha ao chamar o serviço /get_map" << endl;
        exit(1);
    }
}

void Grafo::construirArestas() {
    cout << "Construindo arestas..." << endl;

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

    cout << "Arestas construídas com sucesso." << endl;
}

void Grafo::imprimirGrafo() {
    cout << "\nLista de listaAdjacência:\n";
    for (int id = 0; id < listaAdj.size(); id++) {
        cout << id << " -> ";
        for (int viz : listaAdj[id]) cout << viz << " ";
        cout << endl;
    }
}
