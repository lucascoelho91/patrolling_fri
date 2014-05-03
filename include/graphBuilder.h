/*
 * graphBuilder.h
 *
 *  Created on: Apr 28, 2014
 *      Author: lucas
 */

#ifndef GRAPHBUILDER_H_
#define GRAPHBUILDER_H_

#include <iostream>
#include <vector>
#include <queue>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>


typedef class vertex vertex;
typedef class graph graph;


class rgb  //estrutura usada para representar um pixel
{
    public:
        uint8_t red;
        uint8_t green;
        uint8_t blue;
};

class Vector3
{
	public:
		double x;
		double y;
		double z;
};

class vertex  //vertex é a classe que representa o vértice do grafo
{
    public:
        Vector3  pose;     //posição do nó em metros
        int8_t occupiable;     //indica se o local é ocupável pelo robô (em fronteiras essa variável é zero)
        vertex* neighbor[8]; //ponteiro para os vizinhos do vértice

};


class graph  //graph é a classe que representa o grafo e carrega variáveis sobre o mapa da região
{
    private:
		std::vector < std::vector <vertex*> > matrixGraph; //matriz de vértices que é o grafo

        rgb** colors;        //colors é o mapa da região da forma que ele foi copiado do arquivo de entrada
        rgb** visualization; //essa variável é uma cópia de colors e serve para ser editada para incluir os pixels para visualização da região
        rgb** image;

        Vector3 dim;
        Vector3 sizeMap; //tamanho em metros do mapa

        Vector3 vertices;

        float squareSize;        //tamanho do quadrado que um vértice ocupa, EM PIXELS
        int sizeMetersPixel;

        int8_t drawLines; //especifica śe é para desenhar linhas na visualização do grafo montado

        FILE* mapFile;

    public:
        void BuildGraph(int threshold);        //monta o grafo, lendo o mapa de entrada, inicializando os vértices e estabelecendo os vizinhos

        void ClearGraph(); //limpa o grafo, zerando os valores de owner, visited, have_robot, entre outros

        vertex* coord_to_cell(double y, double x);          //dada uma posição, o programa responde o vértice que esse ponto está contido

        graph (double sizeDiscretization, char* file, double sizeMetersPixel);

        void openMapFile(char* file);
        void printFile();
        void readMap(uint8_t threshold);
        void verticesAllocation();
        void connectNeighbors();

        void DrawSquare(int size, int i, int j, rgb pixel);

};


#endif /* GRAPHBUILDER_H_ */
