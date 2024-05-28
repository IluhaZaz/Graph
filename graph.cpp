#pragma once

#include <vector>
#include <iostream>
#include <set>
#include <iterator>
#include <algorithm>
#include <limits>
#include <map>
#include <exception>

using namespace std;

template<typename Vertex, typename Distance = double>
struct Edge {
    Vertex from;
    Vertex to;
    Distance dist;

    Edge(Vertex from, Vertex to) : from(from), to(to), dist(0) {};
    Edge(Vertex from, Vertex to, Distance dist) : from(from), to(to), dist(dist) {};
};

template<typename Vertex, typename Distance>
bool equals(const Edge<Vertex, Distance>& left, const Edge<Vertex, Distance>& right, bool with_dist = false) {
    if(with_dist)
        return (left.from == right.from) && (left.to == right.to) && (left.dist == right.dist);
    return (left.from == right.from) && (left.to == right.to);
}

template<typename Vertex, typename Distance = double>
class Graph {
private:
    vector<Vertex> _vertices;
    vector<Edge<Vertex, Distance>> _edges;
public:
    Graph() : _edges(vector<Edge<Vertex, Distance>>()), _vertices(vector<Vertex>()) {};

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

        for (int i = 0; i < _vertices.size(); i++) {
            if (_vertices[i] == v) {
                _vertices.erase(_vertices.begin() + i);
                break;
            }
        }

        for (int i = 0; i < _edges.size(); i++) {
            if (_edges[i].to == v || _edges[i].from == v) {
                remove_edge(_edges[i]);
                i--;
            }
        }
        return true;
    }

    vector<Vertex> vertices() const {
        return _vertices;
    }

    vector<Vertex> vertices(const Vertex& start) const {

        vector<Vertex> res;

        vector<Edge<Vertex, Distance>> e = edges(start);

        for (const auto& edge : e) {
            res.push_back(edge.to);
        }

        return res;
    }


    //проверка-добавление-удаление ребер
    void add_edge(const Vertex& from, const Vertex& to,
        const Distance& d, bool oriented = true) {
        Edge<Vertex, Distance> e(from, to, d);
        if (has_vertex(from) && has_vertex(to))
            _edges.push_back(e);
        if (!oriented) {
            if (has_vertex(from) && has_vertex(to))
                _edges.push_back(Edge<Vertex, Distance> (to, from, d));
        }
    }

    bool remove_edge(const Vertex& from, const Vertex& to) {
        for (int i = 0; i < _edges.size(); i++) {
            if (_edges[i].from == from && _edges[i].to == to)
                _edges.erase(_edges.begin() + i);
                return true;
        }
        return false;
    }

    //c учетом рассто€ни€
    bool remove_edge(const Edge<Vertex, Distance>& e) {
        for (int i = 0; i < _edges.size(); i++) {
            if (_edges[i].from == e.from && _edges[i].to == e.to && _edges[i].dist == e.dist)
                _edges.erase(_edges.begin() + i);
            return true;
        }
        return false;
    }

    bool has_edge(const Vertex& from, const Vertex& to) const {
        for (const Edge<Vertex, Distance>& edge : _edges) {
            if (edge.from == from && edge.to == to)
                return true;
        }
        return false;
    }

    //c учетом рассто€ни€ в Edge
    bool has_edge(const Edge<Vertex, Distance>& e) {
        for (const Edge<Vertex, Distance>& edge : _edges) {
            if (edge.from == e.from && edge.to == e.to && edge.dist == e.dist)
                return true;
        }
        return false;
    } 

    //получение всех ребер, выход€щих из вершины
    std::vector<Edge<Vertex, Distance>> edges(const Vertex& vertex) const {
        vector<Edge<Vertex, Distance>> res;
        for (auto edge : _edges) {
            if (edge.from == vertex) {
                res.push_back(edge);
            }
        }
        return res;
    }

    Edge<Vertex, Distance> get_edge(const Vertex& from, const Vertex& to) const {
        for (const Edge<Vertex, Distance>& edge : _edges) {
            if (edge.from == from && edge.to == to)
                return edge;
        }
        throw runtime_error("No such edge");
    }

    //пор€док 
    size_t order() const {
        return (size_t)_vertices.size();
    }

    //степень вершины
    size_t degree(const Vertex& v) const {
        size_t res = 0;
        for (const auto& edge : _edges) {
            if (edge.from == v)
                res += 1;
        }
        return res;
    }

    void dfs(const Vertex& start, set<Vertex>& visited) const {
        visited.insert(start);

        cout << start << " ";

        vector<Vertex> neighbours = vertices(start);

        set<Vertex> next;

        std::set_difference(neighbours.begin(), neighbours.end(), visited.begin(), visited.end(),
std::inserter(next, next.begin()));

        for (const auto& v : next) {
            dfs(v, visited);
        }
    }

    //обход
    std::vector<Vertex>  walk(const Vertex& start_vertex)const {
        set<Vertex> visited;

        dfs(start_vertex, visited);

        return vector<Vertex>(visited.begin(), visited.end());
    }

    Distance value(const Vertex& from, const Vertex& to) const {
        for (const auto& edge : _edges) {
            if (edge.to == to && edge.from == from) {
                return edge.dist;
            }
        }
        throw runtime_error("No such value");
    }

    //поиск кратчайшего пути
    void print_path(vector<Edge<Vertex, Distance>>& path) const {
        for (const auto& edge : path) {
            cout << edge.from << "->" << edge.to << '(' << edge.dist << ')' << endl;
        }
    }

    bool have_negative_v() const  {
        for (const auto& edge : _edges) {
            if (edge.dist < 0)
                return true;
        }
        return false;
    }

    pair<map<Vertex, Distance>, map<Vertex, Vertex>>  dijkstras_algorithm(const Vertex& from) const {

        if (have_negative_v())
            throw runtime_error("Can't use with negative vercites");

        vector<Vertex> unvisited = vertices();
        map<Vertex, Distance> shortest;
        map<Vertex, Vertex> path;

        for (const Vertex& v : unvisited) {
            shortest[v] = numeric_limits<Distance>::max();
        }
        shortest[from] = 0;

        while (!unvisited.empty()) {
            Vertex curr_min = unvisited[0];

            for (const Vertex v : unvisited) {
                if (shortest[v] < shortest[curr_min]) {
                    curr_min = v;
                }
            }

            vector<Vertex> neighbors = vertices(curr_min);

            for (const Vertex neighbor : neighbors) {
                Distance tentative_value = shortest[curr_min] + value(curr_min, neighbor);
                if (tentative_value < shortest[neighbor]) {
                    shortest[neighbor] = tentative_value;
                    path[neighbor] = curr_min;
                }
            }

            int i = 0;
            for (; i < unvisited.size(); i++)
                if (unvisited[i] == curr_min)
                    break;
            unvisited.erase(unvisited.begin() + i);
        }

        return pair(shortest, path);
    }

    vector<Edge<Vertex, Distance>> shortest_path(const Vertex& from,
        const Vertex& to) const {

        if (from == to)
            return vector<Edge<Vertex, Distance>>();

        auto p = dijkstras_algorithm(from);
        map<Vertex, Distance> shortest = p.first;
        map<Vertex, Vertex> path = p.second;

        vector<Edge<Vertex, Distance>> res;

        Vertex last = to;
        Vertex pre_last = path[last];

        while (pre_last) {
            res.push_back(get_edge(pre_last, last));
            last = pre_last;
            pre_last = path.at(last);
        }

        res.push_back(get_edge(from, last));

        auto left = res.begin();
        auto right = res.end() - 1;

        for (int i = 0; i < res.size() / 2; i++) {
            Edge<Vertex, Distance> temp = *left;
            *left = *right;
            *right = temp;
            left++;
            right--;
        }

        print_path(res);

        return res;
    }

    //«адача є2

    Vertex find_centroid() {

        map<Vertex, int> table;

        for (const Vertex v : _vertices) {

            Distance eps = numeric_limits<int>::min();

            auto p = dijkstras_algorithm(v);
            map<Vertex, Distance> shortest = p.first;

            for (const auto& pair: shortest) {
                eps = max(eps, pair.second);
            }
            
            table[v] = eps;
        }

        pair<Vertex, int> res = pair(_vertices[0], table[_vertices[0]]);

        for (const auto& pair : table) {
            if (pair.second < res.second)
                res = pair;
        }

        return res.first;
    }
};