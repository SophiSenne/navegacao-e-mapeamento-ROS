#ifndef GRAFO_H
#define GRAFO_H

#include <bits/stdc++.h>
using namespace std;

class Grafo {
private:
    vector<vector<char>> mapa;   
    string nomeArquivo;

    int idContador = 0;
    int numLinhas = 0;
    int numColunas = 0;

    void lerCSV();
    void lerMapaROS();
    void construirArestas();

public:
    pair<int,int> coorRobo;
    pair<int,int> coorAlvo;

    Grafo(int argc, char** argv);

    vector<vector<int>> listaAdj; 
    map<pair<int,int>, int> coordToId; 
    
    void imprimirGrafo();
};

#endif