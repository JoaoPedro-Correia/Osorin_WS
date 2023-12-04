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
