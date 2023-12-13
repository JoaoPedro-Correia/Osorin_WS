import math
import rospy
from std_msgs.msg import String  # Importe o tipo de mensagem apropriado para as coordenadas do caminho


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


def publicar_caminho(caminho):
    rospy.init_node('publicador_caminho', anonymous=True)
    pub = rospy.Publisher('caminho_encontrado', String, queue_size=10)
    rate = rospy.Rate(1)  # Defina a frequência de publicação (1 Hz neste exemplo)

    while not rospy.is_shutdown():
        coordenadas = ""
        for no in caminho:
            coordenadas += f"({no.x}, {no.y}) "

        rospy.loginfo(coordenadas)
        pub.publish(coordenadas)
        rate.sleep()


# Definição manual da matriz
matriz = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
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

