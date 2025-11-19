#ifndef GRAFO_H
#define GRAFO_H

#include <vector>
#include <map>
#include <string>
#include <utility>

class Grafo {
private:
    std::vector<std::vector<char>> mapa;   
    std::string nomeArquivo;
    int idContador = 0;
    int numLinhas = 0;
    int numColunas = 0;
    
    void lerCSV();
    void lerMapaROS();
    void construirArestas();

public:
    std::pair<int, int> coorRobo;
    std::pair<int, int> coorAlvo;
    
    Grafo(int argc, char** argv);
    
    std::vector<std::vector<int>> listaAdj; 
    std::map<std::pair<int, int>, int> coordToId;
    std::map<int, std::pair<int, int>> idToCoord;  // Mapa reverso: ID -> Coordenadas
    
    void imprimirGrafo();
};

#endif