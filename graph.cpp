#pragma once

#include <vector>

using namespace std;

template<typename Vertex, typename Distance = double>
struct Edge {
    Vertex from;
    Vertex to;
    Distance dist;

    Edge(Vertex from, Vertex to) : from(from), to(to), dist(0) {};
    Edge(Vertex from, Vertex to, Distance dist) : from(from), to(to), dist(dist) {};
};

template<typename Vertex, typename Distance = double>
bool operator==(const Edge<Vertex, Distance>& left, const Edge<Vertex, Distance>& right) {
    return (left.from == right.from) && (left.to == right.to) && (left.dist == right.dist);
}

template<typename Vertex, typename Distance = double>
bool operator!=(const Edge<Vertex, Distance>& left, const Edge<Vertex, Distance>& right) {
    return !(left == right);
}

template<typename Vertex, typename Distance = double>
class Graph {
private:
    vector<Vertex> _vertices;
    vector<Edge<Vertex, Distance>> _edges;
public:
    Graph() : _edges(vector<Edge>()), _vertices(vector<Vertex>()) {};

    //проверка-добавление-удаление вершин
    bool has_vertex(const Vertex& v) const {
        for (const auto& vertex : _vertices)
            if (vertex == v)
                return true;
        return false;
    }

    void add_vertex(const Vertex& v) {
        if (!has_vertex(v))
            _vertices.push_back(v);
    }

    bool remove_vertex(const Vertex& v) {
        if (!has_vertex(v))
            return false;
        for (int i = 0; i < _vertices.size()) {
            if (_vertices[i] == v) {
                _vertices.erase(_vertices.begin() + i);
                return true;
            }
        }
    }

    std::vector<Vertex> vertices() const {
        return _vertices;
    }


    //проверка-добавление-удаление ребер
    void add_edge(const Vertex& from, const Vertex& to,
        const Distance& d) {
        Edge<Vertex, Distance> e(from, to, d);
        if (!has_edge(e))
            _edges.push_back(e);
    }

    bool remove_edge(const Vertex& from, const Vertex& to);

    bool remove_edge(const Edge<Vertex, Distance>& e); //c учетом расстояния

    bool has_edge(const Vertex& from, const Vertex& to) const {
        for (const Edge<Vertex, Distance>& edge : _edges) {
            if (edge.from == from && edge.to == to)
                return true;
        }
        return false;
    }

    //c учетом расстояния в Edge
    bool has_edge(const Edge<Vertex, Distance>& e) {
        for (const Edge<Vertex, Distance>& edge : _edges) {
            if (edge.from == from && edge.to == to && edge.dist == e.dist)
                return true;
        }
        return false;
    } 

    //получение всех ребер, выходящих из вершины
    std::vector<Edge<Vertex, Distance>> edges(const Vertex& vertex);

    size_t order() const; //порядок 
    size_t degree(const Vertex& v) const; //степень вершины


    //поиск кратчайшего пути
    std::vector<Edge<Vertex, Distance>> shortest_path(const Vertex& from,
        const Vertex& to) const;
    //обход
    std::vector<Vertex>  walk(const Vertex& start_vertex)const;
};