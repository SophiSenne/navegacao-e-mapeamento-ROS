#ifndef ALGORITMO_H
#define ALGORITMO_H

#include "Grafo.h"
#include "ControladorRobo.h"
#include <memory>

class Algoritmo {
private:
    Grafo* grafo;
    std::pair<int, int> inicio;
    std::pair<int, int> alvo;

public:
    Algoritmo(Grafo* g, std::pair<int, int> i, std::pair<int, int> a);
    
    void buscaEmLargura();
    void executarBuscaEMovimento(std::shared_ptr<ControladorRobo> controlador);
};

#endif