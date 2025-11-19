#ifndef ALGORITMO_H
#define ALGORITMO_H

#include "Grafo.h"

class Algoritmo {
private:
    pair<int, int> noInicial;
    pair<int, int> alvo;
    Grafo* grafo;
    queue<int> fila;

public:
    Algoritmo(Grafo* grafo, pair<int, int> coorRobo, pair<int, int> coorAlvo);
    void buscaEmLargura();

};

#endif