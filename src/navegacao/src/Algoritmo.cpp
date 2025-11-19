#include "Algoritmo.h"

Algoritmo::Algoritmo(Grafo* grafo, pair<int, int> coorRobo, pair<int, int> coorAlvo) 
    : noInicial(coorRobo), alvo(coorAlvo), grafo(grafo) {

}

void Algoritmo::buscaEmLargura(){
    int idInicio = grafo->coordToId[noInicial];
    int idAlvo = grafo->coordToId[alvo];

    int vert = grafo->listaAdj.size();
    vector<bool> visitado(vert, false);
    vector<int> pai(vert, -1);

    queue<int> fila;
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

    vector<int> caminho;
    for (int v = idAlvo; v != -1; v = pai[v])
        caminho.push_back(v);

    reverse(caminho.begin(), caminho.end());

    cout << "Caminho encontrado (IDs): ";
    for (int id : caminho) cout << id << " ";
    cout << endl;

}