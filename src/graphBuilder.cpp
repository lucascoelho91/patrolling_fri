//============================================================================
// Name        : graphBuilder.cpp
// Author      : Lucas Coelho Figueiredo
// Version     : 2.0
// Copyright   : Do whatever you want with this code!
// Description : This program builds a map/graph using a pgm file as input
//============================================================================

#include "patrolling_fri/graphBuilder.h" 

int coord(int x, int y) //dá o valor do vizinho, de 0 a 7, sendo 0 ao norte, 1 ao nordeste, etc.
{
    if(y==1 && x==-1) return 7;
    else if(y==1) return (x);
    else if(y==-1) return(4-x);
    else if(y==0) return(4-2*x);
    else return 0;
}

void graph::openMapFile(char* file)
{
    mapFile = fopen(file, "r");
}

void graph::readMap(uint8_t threshold)
{
	int i, j, k;
	uint8_t c;
    for(i=0; i<dim.y; i++)
    {
        for(j=0; j<dim.x; j++)
        {
            fscanf( mapFile, "%hhu", &c);   //CAPTURA DA INFORMAÇÃO

            image[i][j].red=c;     //COPIA O VALOR DA COR PARA A MATRIZ DE CORES, UTILIZADA PARA VISUALIZAÇÃO DO GRAFO
            image[i][j].green=c;
            image[i][j].blue=c;
        }

    }
    fclose(mapFile);


    for(i=0, j=dim.y; i<dim.y; i++, j--)
    {
        for(k=0; k<dim.x; k++)
        {
            c=image[j][k].red;
            visualization[i][k].red=c;
            visualization[i][k].green=c;
            visualization[i][k].blue=c;

            colors[i][k].red=c;
            colors[i][k].green=c;
            colors[i][k].blue=c;
            if(c<threshold)             
            {
                matrixGraph[i/squareSize][k/squareSize]=NULL;              //CÉLULA OCUPADA SENDO ANULAD
            }
        }
    }

    for(i=0;i<dim.y; i++)
    {
        free(image[i]);
    }
    free(image);
}

void graph::printFile()
{
    FILE* outfile;
    char* str = (char*) malloc(100*sizeof(char));

    sprintf(str, "result.ppm");
    outfile=fopen(str, "w+");
    fprintf(outfile, "P6\n");   //P6 SIGNIFICA CODIFICAÇÃO CRU, COLORIDO, ARQUIVO PPM
    fprintf(outfile, "%d %d\n", (int) dim.x, (int) dim.y);
    fprintf(outfile, "255\n");  //ESSE NUMERO REPRESENTA O NUMERO MÁXIMO QUE REPRESENTA A COR. DESSA FORMA, 255 255 255 SIGNIFICA BRANCO
    for(int i=(dim.y-1); i>=0; i--)
    {
        for(int j=0; j<dim.x; j++)
        {
            fprintf(outfile, "%c", visualization[i][j].red);
            fprintf(outfile, "%c", visualization[i][j].green);
            fprintf(outfile, "%c", visualization[i][j].blue);
        }
    }
    //printf("Vertices gerados: %ld  Arestas: %ld\n\n\n", num_vertices, num_arestas);
    fclose(outfile);
    free(str);
    return;
}

void graph::DrawSquare(int size, int i, int j, rgb pixel)
{
    int x, y, k, l;
    if(size==0)
    {
        visualization[i][j]=pixel;
        return;
    }

    for(y = i-size, k=0; k<size*2 && y<dim.y; k++, y++)
    {
        for(x = j-size, l=0; l<size*2 && x<dim.x; l++, x++)
        {
            visualization[y][x]=pixel;
        }
    }
}

void graph::verticesAllocation()
{
	int i, j;
	vertex* node;
    rgb pixelred, pixelblue;
        pixelred.red=255;
        pixelred.green=0;
        pixelred.blue=0;

        pixelblue.red=0;
        pixelblue.green=0;
        pixelblue.blue=255;

    for(i=0; i<vertices.y; i++)
    {
        for(j=0; j<vertices.x; j++)
        {
            if(matrixGraph[i][j]!=NULL)     //SE NÃO É NULO, A CÉLULA PASSOU PELO TESTE E VAI RECEBER UM ELEMENTO
            {
                matrixGraph[i][j]=(vertex*) malloc(sizeof(vertex));
                node=matrixGraph[i][j];
                node->occupiable=1;
                node->pose.x= sizeMetersPixel*(j*squareSize + squareSize/2);
                node->pose.y= sizeMetersPixel*(i*squareSize + squareSize/2);

                //COLORIR A IMAGEM DE SAÍDA COM UM PONTO AZUL ONDE HÁ UMA CÉLULA OCUPÁVEL

                DrawSquare(1, matrixGraph[i][j]->pose.x, matrixGraph[i][j]->pose.y, pixelblue);
            }
            else
            {
                //SE É NULO, NÃO É ALOCADO UMA CÉLULA E A IMAGEM RECEBE UM PONTO VERMELHO NO LOCAL DA CÉLULA

                DrawSquare(1, i*squareSize + squareSize/2 , j*squareSize + squareSize/2 , pixelred);
            }
        }
    }
}

void graph::connectNeighbors()
{
	rgb pixelgreen;
	pixelgreen.red=0;
	pixelgreen.green=255;
	pixelgreen.blue=0;
    int i, j, k, l, x, y, c, BLOCK = 2;
    for (i=0; i<vertices.y; i++)
    {
        for(j=0; j<vertices.x; j++)
        {
            if(matrixGraph[i][j]!=NULL)
            {
                for(k=-1; k<=1; k++)
                {
                    for(l=-1; l<=1; l++)
                    {
                        if((i+k)>=0 && (j+l)>=0 && (i+k)<vertices.y && (j+l)<vertices.x)
                        {
                            x=coord(k,l);  //CALCULA O NÚMERO DO VIZINHO DO VÉRTICE
                            if (matrixGraph[(i+k)][(j+l)]!=NULL)
                            {
                                matrixGraph[i][j]->neighbor[x]=matrixGraph[i+k][j+l];
                                    for(y=(i*squareSize+squareSize/2+k*2), x=(j*squareSize+squareSize/2+l*2), c=0; c< (squareSize-2*BLOCK); c++, x=x+l, y=y+k)
                                    {
                                        visualization[y][x]=pixelgreen;
                                    }
                                    //DANG! 8 FORS ONE INSIDE ANOTHER? IS THIS TRIP REALLY NECESSARY? https://www.youtube.com/watch?v=utS4m6I8SCM
                            }
                            else
                            {
                                matrixGraph[i][j]->neighbor[x]=0;
                            }
                        }
                    }
                }
            }
        }
    }
}

graph::graph (double sizeDiscretization, char* file, double sizeMetersPixel)
{
	int i, j;
	openMapFile(file);

    squareSize = sizeDiscretization;
    char* str = (char*) malloc(100*sizeof(char));
    //OS FGETS CAPTURAM AS INFORMAÇÕES QUE NÃO SERÃO UTILIZADAS NO PROGRAMA


    fgets(str, 100, mapFile);    //AQUI É CAPTURADO O TIPO DE CODIFICAÇÃO
    fgets(str, 100, mapFile);    //AQUI A LINHA DE COMENTÁRIO QUE O GIMP DEIXA

    fscanf (mapFile, "%lf", &dim.x);
    fscanf (mapFile, "%lf", &dim.y);


    fgets(str, 100, mapFile);   //AQUI O VALOR MÁXIMO DE CADA BYTE
    fgets(str, 100, mapFile);   //AQUI UM \N QUE INDICA O COMEÇO DA IMAGEM

    vertices.y = dim.y/squareSize;
    vertices.x = dim.x/squareSize;


    /***** ALOCAÇÃO DAS VARIÁVEIS PRINCIPAIS ******************/


    vertex vertexToPoint; //É UMA VARIÁVEL VERTEX UTILIZADA SOMENTE PARA NÃO APONTARMOS PARA UMA COISA QUE NÃO SEJA UM VERTEX
                            //TODAS OS VÉRTICES ALOCADOS DE vertex*** graph APONTARÃO PRA vertexToPoint

    colors=(rgb**) malloc(dim.y*sizeof(rgb*));
    for(i=0; i<dim.y; i++)
    {
        colors[i]=(rgb*) malloc(dim.x*sizeof(rgb));
    }

    visualization=(rgb**) malloc(dim.y*sizeof(rgb*));
    for(i=0; i<dim.y; i++)
    {
        visualization[i]=(rgb*) malloc(dim.x*sizeof(rgb));
    }

    for(i=0;i<=vertices.y;i++)
    {
        std::vector <vertex*> row;
        for(j=0; j<=vertices.x; j++)
        {
            row.push_back(&vertexToPoint);
        }
        matrixGraph.push_back(row);
    }

    image=(rgb**)malloc(dim.y*sizeof(rgb*));
    for(i=0; i<dim.y; i++)
    {
        image[i]=(rgb*)malloc(dim.x*sizeof(rgb));
    }

}

void graph::BuildGraph(int threshold=250)
{
    readMap(threshold);

    verticesAllocation();

    connectNeighbors();

    printFile();
}


int main( int argc, const char* argv[] )
{
	if(argc != 5)
	{
		printf("Invalid arguments. Usage: \\.graphBuilder < pgm map file > < discretization > < map resolution >");
		return 1;
	}

	char mapFile[255];
	strcpy(mapFile, argv[1]);
	printf("%s\n", argv[1]);
	assert(mapFile != NULL && strcmp(mapFile, "\0"));

	double discretization = strtod(argv[2], NULL);
	assert(discretization != 0.0 && errno!=ERANGE);
	printf("%s\n", argv[2]);

	double resolution = strtod(argv[3], NULL);
	assert(resolution != 0.0 && errno!=ERANGE);
	printf("%s\n", argv[3]);

	int threshold = (int) strtol(argv[4], NULL, 10);
	assert(threshold != 0 && errno!=ERANGE);
	printf("%s\n", argv[4]);

	graph Grafo(discretization, mapFile, resolution);
	Grafo.BuildGraph(threshold);

	return 0;

}
