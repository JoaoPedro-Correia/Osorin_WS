#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

class Node {
public:
  int x;
  int y;
  double f;
  double g;
  double h;
  std::vector<Node *> neighbors;

  double heuristic(const Node &goal) const {
    return std::sqrt(std::pow(x - goal.x, 2) + std::pow(y - goal.y, 2));
  }
};

class AStarAlgorithm {
public:
  static std::vector<Node *> aStar(Node *start, Node *goal,
                                   std::vector<std::vector<int>> &matrix,
                                   std::vector<std::vector<Node *>> &nodes) {
    std::vector<Node *> openSet;
    std::unordered_map<Node *, Node *> cameFrom;
    std::unordered_map<Node *, double> gScore;

    openSet.push_back(start);
    gScore[start] = 0.0;
    start->h = start->heuristic(*goal);
    start->f = start->h;

    while (!openSet.empty()) {
      auto current = *std::min_element(
          openSet.begin(), openSet.end(),
          [](const Node *a, const Node *b) { return a->f < b->f; });
      openSet.erase(std::remove(openSet.begin(), openSet.end(), current),
                    openSet.end());

      if (current == goal) {
        return reconstructPath(cameFrom, start, goal);
      }

      for (auto neighbor : current->neighbors) {
        if (neighbor && matrix[neighbor->x][neighbor->y] == 0) {
          double tentativeGScore = gScore[current] + 1;

          if (tentativeGScore < gScore[neighbor]) {
            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeGScore;
            neighbor->h = neighbor->heuristic(*goal);
            neighbor->f = neighbor->h + tentativeGScore;

            if (std::find(openSet.begin(), openSet.end(), neighbor) ==
                openSet.end()) {
              openSet.push_back(neighbor);
            }
          }
        }
      }
    }

    return std::vector<Node *>();
  }

  static std::vector<Node *>
  reconstructPath(std::unordered_map<Node *, Node *> &cameFrom, Node *start,
                  Node *goal) {
    std::vector<Node *> path;
    auto current = goal;

    while (current != start) {
      path.push_back(current);
      current = cameFrom[current];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    return path;
  }
};

int main() {
  // Definição manual da matriz
  std::vector<std::vector<int>> matrix = {
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
      {1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1},
      {1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
      {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1},
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
      // ... resto da matriz ...
  };

  // Criação dos nós a partir da matriz
  int rows = matrix.size();
  int cols = matrix[0].size();
  std::vector<std::vector<Node *>> nodes(rows,
                                         std::vector<Node *>(cols, nullptr));

  // Estabelecimento das conexões entre os nós
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (matrix[i][j] == 0) {
        nodes[i][j] = new Node();
        nodes[i][j]->x = i;
        nodes[i][j]->y = j;

        if (i > 0 && nodes[i - 1][j]) {
          nodes[i][j]->neighbors.push_back(nodes[i - 1][j]); // Acima
          nodes[i - 1][j]->neighbors.push_back(nodes[i][j]); // Abaixo
        }

        if (i < rows - 1 && nodes[i + 1][j]) {
          nodes[i][j]->neighbors.push_back(nodes[i + 1][j]); // Abaixo
          nodes[i + 1][j]->neighbors.push_back(nodes[i][j]); // Acima
        }

        if (j > 0 && nodes[i][j - 1]) {
          nodes[i][j]->neighbors.push_back(nodes[i][j - 1]); // Esquerda
          nodes[i][j - 1]->neighbors.push_back(nodes[i][j]); // Direita
        }

        if (j < cols - 1 && nodes[i][j + 1]) {
          nodes[i][j]->neighbors.push_back(nodes[i][j + 1]); // Direita
          nodes[i][j + 1]->neighbors.push_back(nodes[i][j]); // Esquerda
        }
      }
    }
  }

  // Definição dos nós de início e fim
  int startX = 4, startY = 2; // Coordenadas de início
  int goalX = 9, goalY = 10;  // Coordenadas de chegada

  Node *start = nodes[startX][startY];
  Node *goal = nodes[goalX][goalY];

  // Encontrar o caminho usando A*
  std::vector<Node *> path = AStarAlgorithm::aStar(start, goal, matrix, nodes);

  // Exibir o caminho encontrado
  if (!path.empty()) {
    std::cout << "Caminho encontrado: ";
    for (auto node : path) {
      std::cout << "(" << node->x << ", " << node->y << ") ";
    }
    std::cout << std::endl;
  } else {
    std::cout << "Caminho não encontrado." << std::endl;
  }

  // Liberar memória alocada para os nós
  for (auto &row : nodes) {
    for (auto &node : row) {
      delete node;
    }
  }

  return 0;
}

/*
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.f = 0
        self.g = 0
        self.h = 0
        self.vizinhos = []

    def heuristica(self, objetivo):
        return math.sqrt((self.x - objetivo.x) ** 2 + (self.y - objetivo.y) ** 2)

class AlgoritmoAStar:
    @staticmethod
    def a_estrela(inicio, objetivo, matriz, nos):
        abertos = []
        veio_de = {}
        custo_g = {}

        abertos.append(inicio)
        custo_g[inicio] = 0.0
        inicio.h = inicio.heuristica(objetivo)
        inicio.f = inicio.h

        while abertos:
            atual = min(abertos, key=lambda no: no.f)
            abertos.remove(atual)

            if atual == objetivo:
                return AlgoritmoAStar.reconstruir_caminho(veio_de, inicio, objetivo)

            for vizinho in atual.vizinhos:
                if vizinho and matriz[vizinho.x][vizinho.y] == 0:
                    custo_g_tentativo = custo_g.get(atual, math.inf) + 1

                    if custo_g_tentativo < custo_g.get(vizinho, math.inf):
                        veio_de[vizinho] = atual
                        custo_g[vizinho] = custo_g_tentativo
                        vizinho.h = vizinho.heuristica(objetivo)
                        vizinho.f = vizinho.h + custo_g_tentativo
                        abertos.append(vizinho)

        return []

    @staticmethod
    def reconstruir_caminho(veio_de, inicio, objetivo):
        caminho = []
        atual = objetivo
        while atual != inicio:
            caminho.append(atual)
            atual = veio_de[atual]
        caminho.append(inicio)
        caminho.reverse()
        return caminho

# Definição manual da matriz
matriz = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
]

# Criando nós a partir da matriz
linhas = len(matriz)
colunas = len(matriz[0])
nos = [[Node(i, j) if matriz[i][j] == 0 else None for j in range(colunas)] for i in range(linhas)]

# Estabelecendo conexões entre os nós
for i in range(linhas):
    for j in range(colunas):
        if nos[i][j]:
            if i > 0 and nos[i - 1][j]:
                nos[i][j].vizinhos.append(nos[i - 1][j])  # Acima
            if i < linhas - 1 and nos[i + 1][j]:
                nos[i][j].vizinhos.append(nos[i + 1][j])  # Abaixo
            if j > 0 and nos[i][j - 1]:
                nos[i][j].vizinhos.append(nos[i][j - 1])  # Esquerda
            if j < colunas - 1 and nos[i][j + 1]:
                nos[i][j].vizinhos.append(nos[i][j + 1])  # Direita

# Definindo nó de início e destino
inicio_x, inicio_y = 4, 2  # Coordenadas de início
objetivo_x, objetivo_y = 9, 10  # Coordenadas de chegada

inicio = nos[inicio_x][inicio_y]
objetivo = nos[objetivo_x][objetivo_y]

# Encontrando o caminho usando A*
caminho = AlgoritmoAStar.a_estrela(inicio, objetivo, matriz, nos)

# Exibindo o caminho encontrado
if caminho:
    print("Caminho encontrado:", [(no.x, no.y) for no in caminho])
else:
    print("Caminho não encontrado.")

*/
