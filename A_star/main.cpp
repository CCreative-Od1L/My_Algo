#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include <array>
#include <queue>
#include <map>
#include <cmath>
#include <windows.h>
using namespace std;

#define Vector2 array<int, 2>

constexpr int X = 0;
constexpr int Y = 1;

enum POINT_TYPE {
    EMPTY = 0,
    Start = 1,
    Target,
    Block,
    Run,
    Path,
};

// 计算距离（预估代价）
int heuristic(array<int, 2> a, array<int, 2> b) {
    return abs(a.at(X) - b.at(X)) + abs(a.at(Y) - b.at(Y));
}

bool compare_priority(const array<int, 3> &l_arr, const array<int, 3> &r_arr) {
    return l_arr.at(2) > r_arr.at(2);
}

class GridMap {
public:
    GridMap() = default;
    ~GridMap() = default;
    GridMap(int val_row, int val_col) : row_count(val_row), column_count(val_col) {
        for(auto row = 0; row < val_row; ++row) {
            grid_map.emplace_back(column_count, 0);
        }
    }

    void Print() {
        for_each(grid_map.cbegin(), 
                 grid_map.cend(),
                 [] (const vector<int> &col) {
                    for_each(col.cbegin(), 
                             col.cend(),
                             [] (const int val) {
                                cout << val << " ";
                             });
                    cout << endl;
                 });
        cout << endl;
    }

    // * x, y -> 要求已经经过处理（范围 0 ~ n)
    void UpdatePosition(int x, int y, POINT_TYPE point_type) {
        try {
            grid_map.at(y).at(x) = point_type;
            if (point_type == POINT_TYPE::Start) {
                start_pos = array<int, 2>{x , y};
            }
            if (point_type == POINT_TYPE::Target) {
                target_pos = array<int, 2>{x , y};
            }
        } catch(const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }
    // * y_val -> 要求已经经过变化（范围 0 ~ n)
    bool CheckYIsInRange(int y_val) {
        return (y_val >= 0 && y_val < row_count);
    }
    // * x_val -> 要求已经经过变化（范围 0 ~ n)
    bool CheckXIsInRange(int x_val) {
        return (x_val >= 0 && x_val < row_count);
    }

    // * position -> 要求已经经过变化（范围 0 ~ n)
    vector<array<int, 2>> GetPointNeighbor(const array<int, 2> position) {
        int x_pos = position.at(X);
        int y_pos = position.at(Y);
        vector<array<int, 2>> res;
        if (CheckXIsInRange(x_pos - 1)
            && grid_map.at(y_pos).at(x_pos - 1) != POINT_TYPE::Block) {
                array<int, 2> tmp {x_pos - 1, y_pos};
                res.emplace_back(tmp);
            }
        if (CheckYIsInRange(y_pos - 1)
            && grid_map.at(y_pos - 1).at(x_pos) != POINT_TYPE::Block) {
                array<int,2> tmp {x_pos, y_pos - 1};
                res.emplace_back(tmp);
            }
        if (CheckXIsInRange(x_pos + 1)
            && grid_map.at(y_pos).at(x_pos + 1) != POINT_TYPE::Block) {
                array<int, 2> tmp {x_pos + 1, y_pos};
                res.emplace_back(tmp);
            }
        if (CheckYIsInRange(y_pos + 1)
            && grid_map.at(y_pos + 1).at(x_pos) != POINT_TYPE::Block) {
                array<int, 2> tmp {x_pos, y_pos + 1};
                res.emplace_back(tmp);
            }
        return res;
    }

    void A_Star_Search() {
        priority_queue<array<int,3>, vector<array<int, 3>>, decltype(compare_priority)*> frontier(compare_priority);
        frontier.push(array<int,3>{start_pos.at(X), start_pos.at(Y), 0});
        map<Vector2, Vector2> came_from;
        map<Vector2, int> cost_so_far;

        came_from.insert(make_pair(start_pos, Vector2{-1,-1}));
        cost_so_far.insert(make_pair(start_pos, 0));

        while (!frontier.empty())
        {
            auto current = frontier.top();
            frontier.pop();
            Vector2 current_point{current.at(X), current.at(Y)};
            if (grid_map.at(current_point.at(Y)).at(current_point.at(X)) == POINT_TYPE::Target) {
                break;
            }
            // * Debug
            /*
            if (grid_map.at(current_point.at(Y)).at(current_point.at(X)) == POINT_TYPE::EMPTY) {
                UpdatePosition(current.at(X), current.at(Y), POINT_TYPE::Run);
            }
            */
            auto neighbor_points = GetPointNeighbor(current_point);
            for(auto next_point : neighbor_points){
                auto new_cost = cost_so_far[current_point] + 1;
                if (cost_so_far.find(next_point) == cost_so_far.end() || new_cost < cost_so_far.at(next_point)) {
                    cost_so_far[next_point] = new_cost;

                    auto priority = new_cost + heuristic(target_pos, next_point);
                    array<int, 3> tmp{next_point.at(X), next_point.at(Y), priority};
                    frontier.emplace(tmp);
                    came_from[next_point] = current_point;
                }
            }
        }

        auto tmp_point = target_pos;
        while(tmp_point != start_pos) {
            UpdatePosition(tmp_point.at(X), tmp_point.at(Y), POINT_TYPE::Path);
            tmp_point = came_from[tmp_point];
        }
        Print();
    }
private:
    int row_count;
    int column_count;
    vector<vector<int>> grid_map;

    Vector2 start_pos;
    Vector2 target_pos;
};

void UpdateBlockPosition(GridMap& my_map,const vector<array<int, 2>> &block_list) {
    for_each(block_list.cbegin(),
             block_list.cend(),
             [&my_map] (const array<int, 2> position) {
                my_map.UpdatePosition(position.at(X) - 1, position.at(Y) - 1, POINT_TYPE::Block);
             });
}

void init_map(
    GridMap &my_map,
    const array<int, 2> start_point,
    const array<int, 2> target_point,
    const vector<array<int, 2>> block_list) {
    
    my_map.UpdatePosition(start_point.at(X) - 1, start_point.at(Y) - 1, POINT_TYPE::Start);
    my_map.UpdatePosition(target_point.at(X) - 1, target_point.at(Y) - 1, POINT_TYPE::Target);
    UpdateBlockPosition(my_map, block_list);
}

int main() {
    GridMap my_map(10, 10);
    // initialize the map.
    array<int, 2> start_pos{2, 3};
    array<int, 2> target_pos{8, 5};
    vector<array<int, 2>> block_list {
        {2, 2},{3, 2},{6, 2},{7, 2},{8, 2},{9, 2},
        {3, 3},{6, 3},{6, 4},{3, 5},{4, 5},{6, 5},
        {3, 6},{7, 6},{8, 6},{5, 8},{6, 8},{1, 9},
        {5, 9},{6, 9},{9, 9},{1, 10},{9, 10}
    };
    init_map(my_map, start_pos, target_pos, block_list);

    // my_map.Print();
    my_map.A_Star_Search();
    return 0;
}