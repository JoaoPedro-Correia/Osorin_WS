#include "cell.h"
#include <stdio.h>
#include <stdlib.h>

// VARIAVEIS GLOBAIS
// LINHA E COLUNA
int lin, col;
// MATRIZ DA ESTRUTURA CRIADA
Cell **matrix;

void printMatrix() {
  int i, j;
  printf("*| ");
  for (i = 0; i < col; i++) {
    printf("%d ", i);
  }
  printf("\n");
  for (i = 0; i <= col; i++) {
    printf("--");
  }
  printf("\n");

  for (i = 0; i < lin; i++) {
    printf("%d| ", i);
    for (j = 0; j < col; j++) {
      printf("%d ", get_val(&matrix[i][j]));
    }
    printf("\n");
  }
}

void printCell(int *cell) { printf("(%d,%d)", cell[0], cell[1]); }

int isValid(int x, int y) {
  // VERIFICA SE O NOH SELECIONADO NÃO É OBSTÁCULO OU ESTÁ FORA DA MATRIZ
  return ((x >= 0 && x < col) && (y >= 0 && y < lin) &&
          (get_val(&matrix[x][y]) != 1));
}

void selectCell(int *cell) {
  printf("x: ");
  scanf("%d", &cell[0]);

  printf("y: ");
  scanf("%d", &cell[1]);

  if (!isValid(cell[0], cell[1])) {
    printf("\nINVALIDO!\n");
    selectCell(cell);
  }
}

int manhattanDistance(int x1, int y1, int x2, int y2) {
  // CALCULA A DISTANCIA DE MANHATTAN
  int h = (x2 - x1) + (y2 - y1);
  // MANTEM ELE POSITIVO
  if (h < 0) {
    return (-1 * h);
  } else {
    return h;
  }
}

void defineCourse(int *start, int *end) {
  printf("Selecione o noh de partida!\n");
  selectCell(start);
  printf("Selecione o noh de chegada!\n");
  selectCell(end);
}

Cell **readMatrix() {
  // Abre o file para leitura
  FILE *file = fopen("mapa.txt", "r");

  // Lê as dimensões da matrix
  fscanf(file, "%d %d", &lin, &col);

  // Aloca dinamicamente a matrix
  Cell **matrix = (Cell **)malloc(lin * sizeof(Cell *));
  for (int i = 0; i < lin; i++) {
    matrix[i] = (Cell *)malloc(col * sizeof(Cell));
  }

  int val;
  // Lê os valores da matrix
  for (int i = 0; i < lin; i++) {
    for (int j = 0; j < col; j++) {
      fscanf(file, "%d", &val);
      init_cell(&matrix[i][j], i, j, val);
    }
  }

  // Fecha o file
  fclose(file);

  return matrix;
}

void set_new_noh(Cell *origin, Cell *c, int *endPoint, Cell *openVector,
                 int *nOpen) {
  int manhattan = manhattanDistance(getX(c), getY(c), endPoint[0], endPoint[1]);

  // MENOR DISTANCIA ENCONTRADA
  if (manhattan < getF(c)) {
    setF(c, manhattan);
    // APONTO PARA O
    set_previous(c, origin);
    openVector[(*nOpen)] = *c;
    (*nOpen)++;
  }
}

void printCellArray(struct cell *array, int size) {
  printf("Printing Cell Array:\n");
  for (int i = 0; i < size; i++) {
    printf("Cell %d: (%d, %d) - %d\n", i, array[i].point[0], array[i].point[1],
           array[i].f);
  }
}

int samePoint(int *arr1, int *arr2) {
  return arr1[0] == arr2[0] && arr1[1] == arr2[1];
}

int isPointerInArray(Cell *array, int size, Cell *pointer) {
  for (int i = 0; i < size; i++) {
    if (samePoint(getPoint(&array[i]), getPoint(pointer))) {
      return 0; // Pointer found in the array
    }
  }
  return 1; // Pointer not found in the array
}

// ATUALIZA O CAMINHO DA MATRIZ
void update_new_noh(Cell *c, int *endPoint, Cell *open, int *nOpen,
                    Cell *closeVector, int nClose) {
  // VERIFICA O NOH X+1
  if (isValid(getX(c) + 1, getY(c)) &&
      isPointerInArray(closeVector, nClose, &matrix[getX(c) + 1][getY(c)])) {
    set_new_noh(c, &matrix[getX(c) + 1][getY(c)], endPoint, open, nOpen);
  }
  // VERIFICA O NOH Y+1
  if (isValid(getX(c), getY(c) + 1) &&
      isPointerInArray(closeVector, nClose, &matrix[getX(c)][getY(c) + 1])) {
    set_new_noh(c, &matrix[getX(c)][getY(c) + 1], endPoint, open, nOpen);
  }
  // VERIFICA O NOH X-7
  if (isValid(getX(c) - 1, getY(c)) &&
      isPointerInArray(closeVector, nClose, &matrix[getX(c) - 1][getY(c)])) {
    set_new_noh(c, &matrix[getX(c) - 1][getY(c)], endPoint, open, nOpen);
  }
  // VERIFICA O NOH Y-1
  if (isValid(getX(c), getY(c) - 1) &&
      isPointerInArray(closeVector, nClose, &matrix[getX(c)][getY(c) - 1])) {
    set_new_noh(c, &matrix[getX(c)][getY(c) - 1], endPoint, open, nOpen);
  }
}

// Remove o valor do array na posição especificada
void removeValueAtPosition(Cell *array, int *size, int position) {
  if (position < 0 || position >= *size) {
    // Posição inválida, não faz nada
    return;
  }

  // Desloca os elementos para preencher a posição removida
  for (int i = position; i < *size - 1; i++) {
    array[i] = array[i + 1];
  }

  // Atualiza o tamanho do array
  (*size)--;
}

void printPathFromCell(Cell *cell) {
  while (cell->previous != NULL) {
    printCell(getPoint(cell));
    cell = cell->previous;
    getchar();
  }
  printf("\n");
}

// PROCURA O PROXIMO CAMINHO NA LISTA ABERTA
Cell *search_next_path(Cell *openVector, int *nOpen, int *posicao) {
  // Encontrar a posição do valor no array
  int i, lessVal = getF(&openVector[0]);
  Cell *c = &openVector[0];

  for (i = 1; i < *nOpen; i++) {
    if (getF(&openVector[i]) < lessVal) {
      *posicao = i;
      lessVal = getF(&openVector[i]);
      c = &openVector[i];
    }
  }
  // Atualizar o tamanho do array

  return c;
}

// FUNCAO DO ALGORITMO
void a_star(Cell **matrix, int *start, int *end) {
  int iCurrent;
  Cell *s = &matrix[start[0]][start[1]];
  Cell *e = &matrix[end[0]][end[1]];
  Cell *current;

  // VTOR FECHADO DOS NOHS VISITADOS
  Cell *closeVector = (Cell *)malloc(lin * col * sizeof(Cell));
  Cell *openVector = (Cell *)malloc(lin * col * sizeof(Cell));

  // QUANTIDADE DE NOHs VISITADOS
  int nClose = 0;
  int nOpen = 0;

  update_new_noh(s, end, openVector, &nOpen, closeVector, nClose);
  closeVector[nClose++] = *s;
	
  //LOOP QUE PERCORRE A MATRIZ
  while (!samePoint(getPoint(&closeVector[nClose - 1]), getPoint(e))) {
	//O PROGRAMA PRINTA AS CELULAS PERCORRIDAS SEM TER CONCLUIDO O ALGORITMO
	//NAO FEITO NENHUM TIPO DE ARMAZENAMENTO DO TRAJETO
    printCell(getPoint(&closeVector[nClose - 1]));
    /*
     * SUBSTITUIR A LINHA DE PRINT POR UMA DE PUBLICAÇÃO DO ROS
    */
    current = search_next_path(openVector, &nOpen, &iCurrent);

    update_new_noh(current, end, openVector, &nOpen, closeVector, nClose);
    closeVector[nClose++] = *current;
    removeValueAtPosition(openVector, &nOpen, iCurrent);
  }
  printf("\n");
  free(closeVector);
  free(openVector);
}

/*==========================================================*/
int main() {
  // LER MAPA NO ARQUIVO
  matrix = readMatrix();
  printMatrix();

  // NOH INICIAL
  int startPoint[2];
  // NOH DE CHEGADA
  int endPoint[2];

  // INICIALIZA OS NOH DE LARGADA E CHEGADA
  defineCourse(startPoint, endPoint);

  printf("\n");
  printf("Coordenadas escolhidas\n");
  printCell(startPoint);
  printCell(endPoint);

  printf("\n\n Camiho percorrido:\n");
  a_star(matrix, startPoint, endPoint);

  return 0;
}
