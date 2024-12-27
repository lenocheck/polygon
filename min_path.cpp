#include <iostream> // Подключение библиотеки для ввода-вывода
#include <unordered_map> // Подключение библиотеки для работы с хеш-таблицами
#include <vector> // Подключение библиотеки для работы с динамическими массивами
#include <stack> // Подключение библиотеки для работы со стеком (для DFS)
#include <queue> // Подключение библиотеки для работы с очередью и приоритетной очередью (для BFS, Дейкстры и А*)
#include <cmath> // Подключение библиотеки для математических функций
#include <fstream> // Подключение библиотеки для работы с файлами
#include <sstream> // Подключение библиотеки для работы со строковыми потоками
#include <chrono> // Подключение библиотеки для работы с временем
#include <cassert> // Подключение библиотеки для запуска тестовp
#include <algorithm> // Подключение библиотеки для работы с алгоритмами (для использования reverse)

using namespace std; // Использование стандартного пространства имен для упрощения записи

// Структура Node для представления узла графа
struct Node {
    double lon; // Долгота узла
    double lat; // Широта узла

    // Оператор сравнения для узлов
    bool operator==(const Node other) const {
        return this->lon == other.lon && this->lat == other.lat; // Проверяем равенство двух узлов по координатам
    }

    // Оператор неравенства для узлов
    bool operator!=(const Node other) const {
        return !(*this == other); // Проверяем неравенство, используя оператор равенства
    }

    // Оператор меньше для узлов (для использования в приоритетной очереди)
    bool operator<(const Node other) const {
        if (lon != other.lon) // Сравниваем долготы
            return lon < other.lon;
        return lat < other.lat; // Если долготы равны, сравниваем широты
    }
};

// Специализация хеш-функции для структуры Node (для работы с unordered_map<Node>)
namespace std {
    template <>
    struct hash<Node> {
        // Определяем хеш-функцию для узлов, используя их координаты
        size_t operator()(const Node& node) const {
            // Используем XOR для комбинирования хешей долготы и широты
            return hash<double>()(node.lon) ^ hash<double>()(node.lat);
        }
    };
}


// Структура Graph для представления графа
struct Graph {
    // Список смежности графа, где ключ - узел, а значение - вектор пар (соседний узел, вес ребра)
    unordered_map<Node, vector<pair<Node, double>>> adjList;

    // Метод для поиска ближайшего узла
    Node find_closest_node(Node target) {
        Node closest_node = {0.0, 0.0}; // Инициализируем ближайший узел
        double min_distance = numeric_limits<double>::infinity(); // Инициализируем минимальное расстояние

        // Проходим по всем узлам в графе
        for (auto pair: adjList) {
            Node node = pair.first; // Получаем узел из списка смежности
            double distance = sqrt(
                    pow(node.lon - target.lon, 2) + pow(node.lat - target.lat, 2)); // Вычисляем евклидово расстояние

            if (distance < min_distance) { // Если найдено более близкое расстояние
                min_distance = distance; // Обновляем минимальное расстояние
                closest_node = node; // Обновляем ближайший узел
            }
        }

        return closest_node; // Возвращаем ближайший узел
    }

    // Метод для парсинга данных графа из строки
    void parseData(string data) {
        stringstream ss(data); // Создаем строковый поток для разбора данных
        string line;
        while (getline(ss, line)) { // Читаем строки из потока
            stringstream lineStream(line); // Создаем поток для текущей строки
            double lon1, lat1, lon2, lat2, weight;
            char comma;
            // Читаем долготу и широту первого узла, ожидаем запятую и двоеточие
            if (lineStream >> lon1 >> comma >> lat1 && comma == ',' && lineStream.get() == ':') {
                Node node1{lon1, lat1}; // Создаем узел 1
                // Читаем соседние узлы и веса ребер
                while (lineStream >> lon2 >> comma >> lat2 >> comma >> weight) {
                    Node node2{lon2, lat2}; // Создаем узел 2
                    // Добавляем ребро в список смежности для обоих узлов
                    adjList[node1].emplace_back(node2, weight);
                    adjList[node2].emplace_back(node1, weight);
                    // Проверяем, есть ли конец списка соседей (символ ';')
                    if (!(lineStream >> comma) || comma != ';') {
                        break; // Выходим из цикла, если нет
                    }
                }
            }
        }
    }

    // Метод для очистки графа
    void clear() {
        adjList.clear(); // Очищаем список смежности
    }

    // V - кол-во вершин, E - кол-во рёбер
    // Метод для выполнения поиска в ширину (BFS)
    pair<pair<double, int>, vector<Node>> bfs(Node start, Node goal) {
        // Очередь для хранения вершин и их текущих расстояний
        queue<pair<Node, double>> q; // O(1) - создание очереди, пространственная сложность - O(V) - в худшем случае все вершины могут быть добавлены в очередь

        // Хеш-таблицы для отслеживания посещенных вершин, расстояний и родительских вершин
        unordered_map<Node, bool> visited; // O(1) - создание хеш-таблицы, пространственная сложность - O(V)
        unordered_map<Node, double> distance; // O(1) - создание хеш-таблицы, пространственная сложность - O(V)
        unordered_map<Node, Node> parent; // O(1) - создание хеш-таблицы, пространственная сложность - O(V)

        // Начальная установка: добавляем начальную вершину в очередь с расстоянием 0
        q.push({start, 0.0}); // O(1) - добавление в очередь
        visited[start] = true; // O(1) - отмечаем начальную вершину как посещенную
        distance[start] = 0.0; // O(1) - устанавливаем расстояние до начальной вершины как 0
        parent[start] = start; // O(1) - родитель начальной вершины - сама она

        // Основной цикл BFS
        while (!q.empty()) { // O(V + E) - в худшем случае, когда все вершины и ребра будут обработаны
            // Извлекаем текущую вершину и её расстояние из очереди
            Node current = q.front().first; // O(1) - получение текущего узла, пространственная сложность - O(1)
            double dist = q.front().second; // O(1) - получение расстояния, пространственная сложность - O(1)
            q.pop(); // O(1) - удаление из очереди

            // Если текущая вершина совпадает с целевой вершиной, восстанавливаем путь
            if (current == goal) { // O(1) - проверка на совпадение
                vector<Node> path; // Вектор для хранения пути, пространственная сложность - O(P), где P — длина пути
                int edgeCount = 1; // Счетчик ребер, пространственная сложность - O(1)

                // Восстановление пути от goal до start
                for (Node at = goal; at != start; at = parent[at]) { // O(P) - P - длина пути
                    path.push_back(at); // O(1) - добавляем узел в путь
                    edgeCount++; // O(1) - увеличиваем счетчик
                }
                path.push_back(start); // O(1) - добавляем начальную вершину в путь

                // Обращение пути, чтобы он начинался с start и заканчивался goal
                reverse(path.begin(), path.end()); // O(P) - обращение вектора

                // Возвращаем результат: расстояние, количество ребер и путь
                return {{dist, edgeCount}, path}; // O(1) - возвращение результата
            }

            // Исследование соседних вершин текущей вершины
            for (auto neighbor_pair: adjList[current]) { // O(E) - в худшем случае
                Node neighbor = neighbor_pair.first; // O(1) - получение соседнего узла, пространственная сложность - O(1)
                double weight = neighbor_pair.second; // O(1) - получение веса ребра, пространственная сложность - O(1)

                // Если соседняя вершина не была посещена
                if (!visited[neighbor]) { // O(1) - проверка на посещенность
                    visited[neighbor] = true; // O(1) - отмечаем соседнюю вершину как посещенную
                    distance[neighbor] = dist + weight; // O(1) - обновляем расстояние до соседа
                    parent[neighbor] = current; // O(1) - устанавливаем родителя
                    q.push({neighbor, dist + weight}); // O(1) - добавляем соседа в очередь
                }
            }
        }

        // Если целевая вершина не была найдена, возвращаем бесконечное расстояние и пустой путь
        return {{numeric_limits<double>::infinity(), 0},
                {}}; // O(1) - возвращение результата
    }
    // Общая пространственная сложность - O(V), общая временная сложность - O(V + E)

    // Метод для выполнения поиска в глубину (DFS)
    pair<pair<double, int>, vector<Node>> dfs(Node start, Node goal) {
        // Стек для хранения вершин и их текущих расстояний
        stack<pair<Node, double>> s; // O(1) - создание стека, пространственная сложность - O(V) - в худшем случае все вершины могут быть добавлены в стек

        // Хеш-таблицы для отслеживания посещенных вершин, расстояний и родительских вершин
        unordered_map<Node, bool> visited; // O(1) - создание хеш-таблицы, пространственная сложность - O(V)
        unordered_map<Node, double> distance; // O(1) - создание хеш-таблицы, пространственная сложность - O(V)
        unordered_map<Node, Node> parent; // O(1) - создание хеш-таблицы, пространственная сложность - O(V)

        // Начальная установка: добавляем начальную вершину в стек с расстоянием 0
        s.push({start, 0.0}); // O(1) - добавление в стек
        visited[start] = true; // O(1) - отмечаем начальную вершину как посещенную
        distance[start] = 0.0; // O(1) - устанавливаем расстояние до начальной вершины как 0
        parent[start] = start; // O(1) - родитель начальной вершины - сама она

        // Основной цикл DFS
        while (!s.empty()) { // O(V) - в худшем случае, когда все вершины будут обработаны
            // Извлекаем текущую вершину и её расстояние из стека
            Node current = s.top().first; // O(1) - получение текущего узла, пространственная сложность - O(1)
            double dist = s.top().second; // O(1) - получение расстояния, пространственная сложность - O(1)
            s.pop(); // O(1) - удаление из стека

            // Если текущая вершина совпадает с целевой вершиной, восстанавливаем путь
            if (current == goal) { // O(1) - проверка на совпадение
                vector<Node> path; // Вектор для хранения пути, пространственная сложность - O(P), где P — длина пути
                int edgeCount = 1; // Счетчик ребер, пространственная сложность - O(1)

                // Восстановление пути от goal до start
                for (Node at = goal; at != start; at = parent[at]) { // O(P) - P - длина пути
                    path.push_back(at); // O(1) - добавляем узел в путь
                    edgeCount++; // O(1) - увеличиваем счетчик
                }
                path.push_back(start); // O(1) - добавляем начальную вершину в путь

                // Обращение пути, чтобы он начинался с start и заканчивался goal
                reverse(path.begin(), path.end()); // O(P) - обращение вектора

                // Возвращаем результат: расстояние, количество ребер и путь
                return {{dist, edgeCount}, path}; // O(1) - возвращение результата
            }

            // Исследование соседних вершин текущей вершины
            for (auto neighbor_pair: adjList[current]) { // O(E) - в худшем случае
                Node neighbor = neighbor_pair.first; // O(1) - получение соседнего узла, пространственная сложность - O(1)
                double weight = neighbor_pair.second; // O(1) - получение веса ребра, пространственная сложность - O(1)

                // Если соседняя вершина не была посещена
                if (!visited[neighbor]) { // O(1) - проверка на посещенность
                    visited[neighbor] = true; // O(1) - отмечаем соседнюю вершину как посещенную
                    distance[neighbor] = dist + weight; // O(1) - обновляем расстояние до соседа
                    parent[neighbor] = current; // O(1) - устанавливаем родителя
                    s.push({neighbor, dist + weight}); // O(1) - добавляем соседа в стек
                }
            }
        }

        // Если целевая вершина не была найдена, возвращаем бесконечное расстояние и пустой путь
        return {{numeric_limits<double>::infinity(), 0},
                {}}; // O(1) - возвращение результата
    }
    // Общая пространственная сложность - O(V), общая временная сложность - O(V + E)


    // Метод для выполнения алгоритма Дейкстры
    pair<pair<double, int>, vector<Node>> dijkstra(Node start, Node goal) {
        // Приоритетная очередь для хранения узлов и их текущих расстояний (первыми в ней будут элементы с меньшим весом)
        priority_queue<pair<double, Node>, vector<pair<double, Node>>, greater<pair<double, Node>>> pq; // O(1) - создание приоритетной очереди, пространственная сложность - O(V) - в худшем случае все узлы могут быть добавлены в очередь

        // Хеш-таблицы для хранения расстояний, посещенных узлов и родительских узлов
        unordered_map<Node, double> distances; // O(1) - создание хеш-таблицы для расстояний, пространственная сложность - O(V)
        unordered_map<Node, bool> visited; // O(1) - создание хеш-таблицы для посещенных узлов, пространственная сложность - O(V)
        unordered_map<Node, Node> parent; // O(1) - создание хеш-таблицы для родительских узлов, пространственная сложность - O(V)

        // Инициализация расстояний до всех узлов как бесконечность
        for (auto pair: adjList) { // O(V) - инициализация расстояний
            distances[pair.first] = numeric_limits<double>::infinity(); // O(1) - установка расстояния
        }

        // Установка начального расстояния до стартового узла
        distances[start] = 0.0; // O(1) - установка расстояния до стартового узла
        pq.push({0.0, start}); // O(log V) - добавление стартового узла в приоритетную очередь
        parent[start] = start; // O(1) - родитель стартового узла - сама она

        // Основной цикл алгоритма Дейкстры
        while (!pq.empty()) { // O(V) - в худшем случае, когда все узлы будут обработаны
            // Извлечение узла с минимальным расстоянием
            double currentDistance = pq.top().first; // O(1) - получение минимального расстояния, пространственная сложность - O(1)
            Node current = pq.top().second; // O(1) - получение узла, пространственная сложность - O(1)
            pq.pop(); // O(log V) - удаление узла из приоритетной очереди

            // Если текущий узел совпадает с целевым узлом, восстанавливаем путь
            if (current == goal) { // O(1) - проверка на совпадение
                vector<Node> path; // Вектор для хранения пути, пространственная сложность - O(P), где P — длина пути
                int edgeCount = 1; // Счетчик ребер, пространственная сложность - O(1)
                for (Node at = goal; at != start; at = parent[at]) { // O(P) - P - длина пути
                    path.push_back(at); // O(1) - добавляем узел в путь
                    edgeCount++; // O(1) - увеличиваем счетчик
                }
                path.push_back(start); // O(1) - добавляем начальную вершину в путь
                reverse(path.begin(), path.end()); // O(P) - обращение пути
                return {{currentDistance, edgeCount}, path}; // O(1) - возвращение результата
            }

            // Если узел уже посещен, пропускаем его
            if (visited[current]) continue; // O(1) - проверка на посещенность
            visited[current] = true; // O(1) - отмечаем текущий узел как посещенный

            // Исследование соседних узлов
            for (auto neighbor_pair: adjList[current]) { // O(E) - в худшем случае, когда все соседи текущего узла будут обработаны
                Node neighbor = neighbor_pair.first; // O(1) - получение соседнего узла, пространственная сложность - O(1)
                double weight = neighbor_pair.second; // O(1) - получение веса ребра, пространственная сложность - O(1)

                // Вычисление нового расстояния до соседнего узла
                double newDist = currentDistance + weight; // O(1) - вычисление нового расстояния, пространственная сложность - O(1)

                // Если новое расстояние меньше текущего расстояния до соседнего узла
                if (newDist < distances[neighbor]) { // O(1) - проверка условия
                    distances[neighbor] = newDist; // O(1) - обновляем расстояние до соседа
                    parent[neighbor] = current; // O(1) - устанавливаем родителя
                    pq.push({newDist, neighbor}); // O(log V) - добавляем соседа в очередь
                }
            }
        }

        // Если целевой узел не был найден, возвращаем бесконечное расстояние и пустой путь
        return {{numeric_limits<double>::infinity(), 0},
                {}}; // O(1) - возвращение результата
    }
    // Общая пространственная сложность - O(V), общая временная сложность - O((V + E) log V)


    // Эвристическая функция для оценки расстояния между двумя узлами (евклидово расстояние)
    double heuristic(Node a, Node b) {
        return sqrt(pow(a.lon - b.lon, 2) + pow(a.lat - b.lat, 2)); // O(1) - вычисляем евклидово расстояние, пространственная сложность - O(1)
    }

    // Метод для выполнения алгоритма A*
    pair<pair<double, int>, vector<Node>> aStar(Node start, Node goal) {
        // Приоритетная очередь для хранения узлов и их текущих fScore (первыми в ней будут элементы с меньшим весом)
        priority_queue<pair<double, Node>, vector<pair<double, Node>>, greater<pair<double, Node>>> pq; // O(1) - создание приоритетной очереди, пространственная сложность - O(V) - в худшем случае все узлы могут быть добавлены в очередь

        // Хеш-таблицы для хранения gScore, fScore и родительских узлов
        unordered_map<Node, double> gScore; // O(1) - создание хеш-таблицы для gScore, пространственная сложность - O(V)
        unordered_map<Node, double> fScore; // O(1) - создание хеш-таблицы для fScore, пространственная сложность - O(V)
        unordered_map<Node, Node> parent; // O(1) - создание хеш-таблицы для родительских узлов, пространственная сложность - O(V)

        // Инициализация gScore и fScore для всех узлов как бесконечность
        for (auto pair: adjList) { // O(V) - инициализация gScore и fScore
            gScore[pair.first] = numeric_limits<double>::infinity(); // O(1) - установка gScore
            fScore[pair.first] = numeric_limits<double>::infinity(); // O(1) - установка fScore
        }

        // Установка начального gScore и fScore для стартового узла
        gScore[start] = 0.0; // O(1) - gScore для стартового узла
        fScore[start] = heuristic(start, goal); // O(1) - fScore для стартового узла
        parent[start] = start; // O(1) - родитель стартового узла - сама она

        // Добавление стартового узла в приоритетную очередь
        pq.push({fScore[start], start}); // O(log V) - добавление стартового узла в очередь

        // Основной цикл алгоритма A*
        while (!pq.empty()) { // O(V) - в худшем случае, когда все узлы будут обработаны
            // Извлечение узла с минимальным fScore
            double currentFScore = pq.top().first; // O(1) - получение минимального fScore, пространственная сложность - O(1)
            Node current = pq.top().second; // O(1) - получение узла, пространственная сложность - O(1)
            pq.pop(); // O(log V) - удаление узла из приоритетной очереди

            // Если текущий узел совпадает с целевым узлом, восстанавливаем путь
            if (current == goal) { // O(1) - проверка на совпадение
                vector<Node> path; // Вектор для хранения пути, пространственная сложность - O(P), где P — длина пути
                int edgeCount = 1; // Счетчик ребер, пространственная сложность - O(1)
                for (Node at = goal; at != start; at = parent[at]) { // O(P) - P - длина пути
                    path.push_back(at); // O(1) - добавляем узел в путь
                    edgeCount++; // O(1) - увеличиваем счетчик
                }
                path.push_back(start); // O(1) - добавляем начальную вершину в путь
                reverse(path.begin(), path.end()); // O(P) - обращаем путь
                return {{gScore[current], edgeCount}, path}; // O(1) - возвращаем результат
            }

            // Исследование соседних узлов
            for (auto neighbor_pair: adjList[current]) { // O(E) - в худшем случае, когда все соседи текущего узла будут обработаны
                Node neighbor = neighbor_pair.first; // O(1) - получение соседнего узла, пространственная сложность - O(1)
                double weight = neighbor_pair.second; // O(1) - получение веса ребра, пространственная сложность - O(1)

                // Вычисление временного gScore для соседнего узла
                double tentativeGScore = gScore[current] + weight; // O(1) - вычисление временного gScore, пространственная сложность - O(1)

                // Если временный gScore меньше текущего gScore для соседнего узла
                if (tentativeGScore < gScore[neighbor]) { // O(1) - проверка условия
                    // Обновление gScore и fScore для соседнего узла
                    gScore[neighbor] = tentativeGScore; // O(1) - обновляем gScore
                    fScore[neighbor] = tentativeGScore + heuristic(neighbor, goal); // O(1) - обновляем fScore
                    parent[neighbor] = current; // O(1) - устанавливаем родителя
                    pq.push({fScore[neighbor], neighbor}); // O(log V) - добавляем соседа в очередь
                }
            }
        }

        // Если целевой узел не был найден, возвращаем бесконечное расстояние и пустой путь
        return {{numeric_limits<double>::infinity(), 0},
                {}}; // O(1) - возвращение результата
    }
    // Общая пространственная сложность - O(V), общая временная сложность - O((V + E) log V)
};

// Функция для запуска тестов
void runTests() {
    Graph graph; // Создаем экземпляр графа

    // Тест 1: Простой граф с прямым путем
    string data1 = R"(
        1.0,1.0:2.0,2.0,1.0;
        2.0,2.0:3.0,3.0,1.0;
        3.0,3.0:4.0,4.0,1.0;
    )";
    graph.parseData(data1); // Парсим данные графа из строки

    Node start1{ 1.0, 1.0 }; // Определяем начальную вершину
    Node goal1{ 4.0, 4.0 }; // Определяем целевую вершину

    // Проверяем, что все алгоритмы возвращают правильное расстояние
    assert(graph.bfs(start1, goal1).first.first == 3.0); // Проверка BFS
    assert(graph.dfs(start1, goal1).first.first == 3.0); // Проверка DFS
    assert(graph.dijkstra(start1, goal1).first.first == 3.0); // Проверка алгоритма Дейкстры
    assert(graph.aStar(start1, goal1).first.first == 3.0); // Проверка алгоритма A*

    graph.clear(); // Очищаем граф для следующего теста

    // Тест 2: Граф без пути
    string data2 = R"(
        1.0,1.0:2.0,2.0,1.0;
        3.0,3.0:4.0,4.0,1.0;
    )";
    graph.parseData(data2); // Парсим данные графа из строки

    Node start2{ 1.0, 1.0 }; // Определяем начальную вершину
    Node goal2{ 4.0, 4.0 }; // Определяем целевую вершину

    // Проверяем, что все алгоритмы возвращают бесконечное расстояние
    assert(graph.bfs(start2, goal2).first.first == numeric_limits<double>::infinity()); // Проверка BFS
    assert(graph.dfs(start2, goal2).first.first == numeric_limits<double>::infinity()); // Проверка DFS
    assert(graph.dijkstra(start2, goal2).first.first == numeric_limits<double>::infinity()); // Проверка алгоритма Дейкстры
    assert(graph.aStar(start2, goal2).first.first == numeric_limits<double>::infinity()); // Проверка алгоритма A*

    graph.clear(); // Очищаем граф для следующего теста

    // Тест 3: Граф с циклом
    string data3 = R"(
        1.0,1.0:2.0,2.0,1.0;
        2.0,2.0:3.0,3.0,1.0;
        3.0,3.0:1.0,1.0,1.0;
    )";
    graph.parseData(data3); // Парсим данные графа из строки

    Node start3{ 1.0, 1.0 }; // Определяем начальную вершину
    Node goal3{ 3.0, 3.0 }; // Определяем целевую вершину

    // Проверяем, что все алгоритмы возвращают правильное расстояние
    assert(graph.bfs(start3, goal3).first.first == 1.0); // Проверка BFS
    assert(graph.dfs(start3, goal3).first.first == 1.0); // Проверка DFS
    assert(graph.dijkstra(start3, goal3).first.first == 1.0); // Проверка алгоритма Дейкстры
    assert(graph.aStar(start3, goal3).first.first == 1.0); // Проверка алгоритма A*

    cout << "All tests passed!" << endl;
}

// Функция для тестирования данных из файла
void test_file() {
    Graph graph; // Создаем экземпляр графа

    // Чтение данных из файла
    ifstream inputFile("C:/Users/Lena/Downloads/graph.txt"); // Открываем файл для чтения
    if (!inputFile.is_open()) { // Проверяем, был ли файл успешно открыт
        cerr << "Не удалось открыть файл!" << endl;
        return;
    }

    stringstream buffer; // Создаем поток для хранения данных
    buffer << inputFile.rdbuf(); // Считываем содержимое файла в поток
    string data = buffer.str(); // Преобразуем поток в строку
    graph.parseData(data); // Парсим данные графа

    Node start{ 1.0, 1.0 }; // Определяем начальную вершину
    Node goal{ 5.0, 5.0 }; // Определяем целевую вершину

    // Проверяем, что все алгоритмы возвращают правильное расстояние
    assert(graph.bfs(start, goal).first.first == 6.0); // Проверка BFS
    assert(graph.dfs(start, goal).first.first == 6.0); // Проверка DFS
    assert(graph.dijkstra(start, goal).first.first == 6.0); // Проверка алгоритма Дейкстры
    assert(graph.aStar(start, goal).first.first == 6.0); // Проверка алгоритма A*

    // Запуск алгоритмов и замер времени
    auto startBFS = chrono::high_resolution_clock::now(); // Начало замера времени для BFS
    auto bfsResult = graph.bfs(start, goal); // Запуск BFS
    auto endBFS = chrono::high_resolution_clock::now(); // Конец замера времени для BFS
    cout << "BFS Distance: " << bfsResult.first.first << " Edges: " << bfsResult.first.second << " Time: "
         << chrono::duration_cast<chrono::microseconds>(endBFS - startBFS).count() << "ms" << endl; // Выводим результаты BFS
    cout << "BFS Path: "; // Выводим путь, найденный BFS
    for (auto node : bfsResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") "; // Форматируем вывод координат узлов пути
    }
    cout << endl;

    auto startDFS = chrono::high_resolution_clock::now(); // Начало замера времени для DFS
    auto dfsResult = graph.dfs(start, goal); // Запуск DFS
    auto endDFS = chrono::high_resolution_clock::now(); // Конец замера времени для DFS
    cout << "DFS Distance: " << dfsResult.first.first << " Edges: " << dfsResult.first.second << " Time: "
         << chrono::duration_cast<chrono::microseconds>(endDFS - startDFS).count() << "ms" << endl; // Выводим результаты DFS
    cout << "DFS Path: "; // Выводим путь, найденный DFS
    for (auto node : dfsResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") "; // Форматируем вывод координат узлов пути
    }
    cout << endl;

    auto startDijkstra = chrono::high_resolution_clock::now(); // Начало замера времени для алгоритма Дейкстры
    auto dijkstraResult = graph.dijkstra(start, goal); // Запуск алгоритма Дейкстры
    auto endDijkstra = chrono::high_resolution_clock::now(); // Конец замера времени для алгоритма Дейкстры
    cout << "Dijkstra Distance: " << dijkstraResult.first.first << " Edges: " << dijkstraResult.first.second << " Time: "
         << chrono::duration_cast<chrono::microseconds>(endDijkstra - startDijkstra).count() << "ms" << endl; // Выводим результаты алгоритма Дейкстры
    cout << "Dijkstra Path: "; // Выводим путь, найденный алгоритмом Дейкстры
    for (auto node : dijkstraResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") "; // Форматируем вывод координат узлов пути
    }
    cout << endl;

    auto startAStar = chrono::high_resolution_clock::now(); // Начало замера времени для алгоритма A*
    auto aStarResult = graph.aStar(start, goal); // Запуск алгоритма A*
    auto endAStar = chrono::high_resolution_clock::now(); // Конец замера времени для алгоритма A*
    cout << "A* Distance: " << aStarResult.first.first << " Edges: " << aStarResult.first.second << " Time: "
         << chrono::duration_cast<chrono::microseconds>(endAStar - startAStar).count() << "ms" << endl; // Выводим результаты алгоритма A*
    cout << "A* Path: "; // Выводим путь, найденный алгоритмом A*
    for (auto node : aStarResult.second) {
        cout << "(" << node.lon << ", " << node.lat << ") "; // Форматируем вывод координат узлов пути
    }
    cout << endl;

    cout << "Test_file passed!" << endl;
}

int main() {
    // Запускаем тесты, чтобы проверить корректность работы алгоритмов
    runTests();
    // Тестируем обработку данных из файла
    test_file();

    Graph graph; // Создаем экземпляр графа для работы с данными

    // Чтение данных из файла
    ifstream inputFile("C:/Users/Lena/Downloads/spb_graph (1).txt"); // Открываем файл для чтения данных графа
    if (!inputFile.is_open()) { // Проверяем, был ли файл успешно открыт
        cerr << "Не удалось открыть файл!" << endl;
        return 1;
    }

    stringstream buffer; // Создаем поток для хранения данных из файла
    buffer << inputFile.rdbuf(); // Считываем содержимое файла в поток
    string data = buffer.str(); // Преобразуем поток в строку, содержащую данные графа
    graph.parseData(data); // Парсим данные графа из строки

    // Определяем стартовую и целевую вершины для поиска пути
     Node start{ 30.462547, 59.916606 }; // Стартовая вершина
     Node goal{ 30.308108, 59.957238 }; // Целевая вершина

    // Запуск алгоритмов поиска пути и замер времени выполнения
    // Ниже приведены примеры замера времени и вывода результатов для различных алгоритмов.

    // Запуск и замер времени для алгоритма BFS
//     auto startBFS = chrono::high_resolution_clock::now(); // Начало замера времени для BFS
//     auto bfsResult = graph.bfs(graph.find_closest_node(start), graph.find_closest_node(goal)); // Запуск алгоритма BFS для поиска пути от ближайшей к стартовой вершины до ближайшей к целевой вершины из файла
//     auto endBFS = chrono::high_resolution_clock::now(); // Конец замера времени для BFS
    // Выводим результаты BFS и время выполнения в секундах
//     cout << "BFS Distance: " << bfsResult.first.first << " Edges: " << bfsResult.first.second << " Time: "
//          << chrono::duration_cast<chrono::milliseconds>(endBFS - startBFS).count() / 1000.0 << "s" << endl;
//     cout << "BFS Path: "; // Выводим путь, найденный BFS
//     for (auto node : bfsResult.second) {
//         cout << "(" << node.lon << ", " << node.lat << ") "; // Форматируем вывод координат узлов пути
//     }
//     cout << endl;

    // Аналогично для алгоритма DFS
//     auto startDFS = chrono::high_resolution_clock::now(); // Начало замера времени для DFS
//     auto dfsResult = graph.dfs(graph.find_closest_node(start), graph.find_closest_node(goal)); // Запуск алгоритма DFS для поиска пути от ближайшей к стартовой вершины до ближайшей к целевой вершины из файла
//     auto endDFS = chrono::high_resolution_clock::now(); // Конец замера времени для DFS
    // Выводим результаты DFS и время выполнения в секундах
//     cout << "DFS Distance: " << dfsResult.first.first << " Edges: " << dfsResult.first.second << " Time: "
//          << chrono::duration_cast<chrono::milliseconds>(endDFS - startDFS).count() / 1000.0 << "s" << endl;
//     cout << "DFS Path: "; // Выводим путь, найденный DFS
//     for (auto node : dfsResult.second) {
//         cout << "(" << node.lon << ", " << node.lat << ") "; // Форматируем вывод координат узлов пути
//     }
//     cout << endl;

    // Аналогично для алгоритма Дейкстры
//     auto startDijkstra = chrono::high_resolution_clock::now(); // Начало замера времени для алгоритма Дейкстры
//     auto dijkstraResult = graph.dijkstra(graph.find_closest_node(start), graph.find_closest_node(goal)); // Запуск алгоритма Дейкстры для поиска кратчайшего пути от ближайшей к стартовой вершины до ближайшей к целевой вершины из файла
//     auto endDijkstra = chrono::high_resolution_clock::now(); // Конец замера времени для алгоритма Дейкстры
    // Выводим результаты алгоритма Дейкстры и время выполнения в секундах
//     cout << "Dijkstra Distance: " << dijkstraResult.first.first << " Edges: " << dijkstraResult.first.second << " Time: "
//          << chrono::duration_cast<chrono::milliseconds>(endDijkstra - startDijkstra).count() / 1000.0 << "s" << endl;
//     cout << "Dijkstra Path: "; // Выводим путь, найденный алгоритмом Дейкстры
//     for (auto node : dijkstraResult.second) {
//         cout << "(" << node.lon << ", " << node.lat << ") "; // Форматируем вывод координат узлов пути
//     }
//     cout << endl;

    // Аналогично для алгоритма A*
//     auto startAStar = chrono::high_resolution_clock::now(); // Начало замера времени для алгоритма A*
//     auto aStarResult = graph.aStar(graph.find_closest_node(start), graph.find_closest_node(goal)); // Запуск алгоритма A* для поиска кратчайшего пути от ближайшей к стартовой вершины до ближайшей к целевой вершины из файла
//     auto endAStar = chrono::high_resolution_clock::now(); // Конец замера времени для алгоритма A*
    // Выводим результаты алгоритма A* и время выполнения в секундах
//     cout << "A* Distance: " << aStarResult.first.first << " Edges: " << aStarResult.first.second << " Time: "
//          << chrono::duration_cast<chrono::milliseconds>(endAStar - startAStar).count() / 1000.0 << "s" << endl;
//     cout << "A* Path: "; // Выводим путь, найденный алгоритмом A*
//     for (auto node : aStarResult.second) {
//         cout << "(" << node.lon << ", " << node.lat << ") "; // Форматируем вывод координат узлов пути
//     }
//     cout << endl;

    // Запуск алгоритмов поиска пути и замер времени выполнения каждого алгоритма

    // Начало замера времени для алгоритма BFS (поиск в ширину)
    auto startBFS = chrono::high_resolution_clock::now(); // Запоминаем текущее время перед запуском алгоритма
    auto bfsResult = graph.bfs(graph.find_closest_node(start), graph.find_closest_node(goal)); // Запускаем алгоритм BFS для поиска пути от ближайшей к стартовой вершины до ближайшей к целевой вершины из файла
    auto endBFS = chrono::high_resolution_clock::now(); // Запоминаем текущее время после завершения алгоритма

    // Выводим время выполнения алгоритма BFS в секундах
    cout << "BFS Time: "
         << chrono::duration_cast<chrono::milliseconds>(endBFS - startBFS).count() / 1000.0 << "s" << endl; // Преобразуем разницу времени в секунды и выводим

    // Начало замера времени для алгоритма DFS (поиск в глубину)
    auto startDFS = chrono::high_resolution_clock::now(); // Запоминаем текущее время перед запуском алгоритма
    auto dfsResult = graph.dfs(graph.find_closest_node(start), graph.find_closest_node(goal)); // Запускаем алгоритм DFS для поиска пути от ближайшей к стартовой вершины до ближайшей к целевой вершины из файла
    auto endDFS = chrono::high_resolution_clock::now(); // Запоминаем текущее время после завершения алгоритма

    // Выводим время выполнения алгоритма DFS в секундах
    cout << "DFS Time: "
         << chrono::duration_cast<chrono::milliseconds>(endDFS - startDFS).count() / 1000.0 << "s" << endl; // Преобразуем разницу времени в секунды и выводим

    // Начало замера времени для алгоритма Дейкстры
    auto startDijkstra = chrono::high_resolution_clock::now(); // Запоминаем текущее время перед запуском алгоритма
    auto dijkstraResult = graph.dijkstra(graph.find_closest_node(start), graph.find_closest_node(goal)); // Запускаем алгоритм Дейкстры для поиска кратчайшего пути от ближайшей к стартовой вершины до ближайшей к целевой вершины из файла
    auto endDijkstra = chrono::high_resolution_clock::now(); // Запоминаем текущее время после завершения алгоритма

    // Выводим время выполнения алгоритма Дейкстры в секундах
    cout << "Dijkstra Time: "
         << chrono::duration_cast<chrono::milliseconds>(endDijkstra - startDijkstra).count() / 1000.0 << "s" << endl; // Преобразуем разницу времени в секунды и выводим

    // Начало замера времени для алгоритма A*
    auto startAStar = chrono::high_resolution_clock::now(); // Запоминаем текущее время перед запуском алгоритма
    auto aStarResult = graph.aStar(graph.find_closest_node(start), graph.find_closest_node(goal)); // Запускаем алгоритм A* для поиска кратчайшего пути с учетом эвристики от ближайшей к стартовой вершины до ближайшей к целевой вершины из файла
    auto endAStar = chrono::high_resolution_clock::now(); // Запоминаем текущее время после завершения алгоритма

    // Выводим время выполнения алгоритма A* в секундах
    cout << "A* Time: "
         << chrono::duration_cast<chrono::milliseconds>(endAStar - startAStar).count() / 1000.0 << "s" << endl; // Преобразуем разницу времени в секунды и выводим

    return 0;
}
