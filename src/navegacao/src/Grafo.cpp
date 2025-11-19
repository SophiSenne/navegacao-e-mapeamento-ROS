#include "Grafo.h"

#include <bits/stdc++.h>
using namespace std;

Grafo::Grafo(string nomeArquivo) : nomeArquivo(nomeArquivo) {
    lerCSV();
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
