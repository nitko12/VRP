#include <iostream>
#include <cstdio>
#include <vector>
#include <array>
#include <algorithm>
#include <random>
#include <cmath>
#include <cassert>
#include <set>
#include <map>

// #include "kdtree.h"
#include "lp_minimizer.hpp"

using namespace std;
// using namespace kdt;

typedef array<int, 2> point;

// NAME : A-n32-k5
// COMMENT : (Augerat et al, No of trucks: 5, Optimal value: 784)
// TYPE : CVRP
// DIMENSION : 32
// EDGE_WEIGHT_TYPE : EUC_2D
// CAPACITY : 100
// NODE_COORD_SECTION

auto rd = random_device{};
auto rng = default_random_engine{rd()};

class problem
{
private:
    string name, comment, type, edge_weight_type;
    int dimension, capacity;
    vector<point> nodes;
    vector<int> demand, depots;

    LPMinimizer lp_minimizer;

    int prev_node(int node) const
    {
        if (node == 0)
            return nodes.size() - 1;
        return node - 1;
    }
    static int prev_node(int node, int n)
    {
        if (node == 0)
            return n - 1;
        return node - 1;
    }
    int next_node(int node) const
    {
        if (node == nodes.size() - 1)
            return 0;
        return node + 1;
    }
    static int next_node(int node, int n)
    {
        if (node == n - 1)
            return 0;
        return node + 1;
    }

    static double dist(const point &a, const point &b)
    {
        return sqrt((a[0] - b[0]) * (a[0] - b[0]) +
                    (a[1] - b[1]) * (a[1] - b[1]));
    }

    static set<int> generate_random_set(int n, int lb, int ub, set<int> not_in = set<int>())
    {
        set<int> result;

        auto r = uniform_int_distribution<int>(lb, ub);

        while (result.size() < n)
        {
            int x = r(rng);
            if (not_in.find(x) == not_in.end())
                result.insert(x);
        }

        return result;
    }

public:
    void load_problem()
    {
        string s;
        while (cin >> s)
        {
            if (s == "NAME")
            {
                cin >> s;
                cin >> name;
            }
            else if (s == "COMMENT")
            {
                cin >> s;
                getline(cin, comment);
            }
            else if (s == "TYPE")
            {
                cin >> s;
                cin >> type;
            }
            else if (s == "DIMENSION")
            {
                cin >> s;
                cin >> dimension;
            }
            else if (s == "EDGE_WEIGHT_TYPE")
            {
                cin >> s;
                cin >> edge_weight_type;
            }
            else if (s == "CAPACITY")
            {
                cin >> s;
                cin >> capacity;
            }
            else if (s == "NODE_COORD_SECTION")
            {
                for (int i = 0; i < dimension; i++)
                {
                    int t, x, y;
                    cin >> t >> x >> y;

                    nodes.push_back({x, y});
                    // nodes.push_back({x, y});
                }
            }
            else if (s == "DEMAND_SECTION")
            {
                for (int i = 0; i < dimension; i++)
                {
                    int t, d;
                    cin >> t >> d;
                    demand.push_back(d);
                }
            }
            else if (s == "DEPOT_SECTION")
            {
                int d;
                while (cin >> d, d != -1)
                {
                    depots.push_back(d - 1);
                }
            }
            else if (s == "EOF")
            {
                break;
            }
            else
            {
                cout << "no" << endl;
            }
        }
    };

    void print_problem()
    {
        cout << "NAME : " << name << endl;
        cout << "COMMENT : " << comment << endl;
        cout << "TYPE : " << type << endl;
        cout << "DIMENSION : " << dimension << endl;
        cout << "EDGE_WEIGHT_TYPE : " << edge_weight_type << endl;
        cout << "CAPACITY : " << capacity << endl;
        cout << "NODE_COORD_SECTION" << endl;
        for (int i = 0; i < dimension; i++)
        {
            cout << i << " " << nodes[i][0] << " " << nodes[i][1] << endl;
        }
        cout << "DEMAND_SECTION" << endl;
        for (int i = 0; i < dimension; i++)
        {
            cout << i << " " << demand[i] << endl;
        }
        cout << "DEPOT_SECTION" << endl;
        for (int i = 0; i < depots.size(); i++)
        {
            cout << depots[i] << " ";
        }
        cout << endl;
    }

    vector<int> random_solve()
    {
        vector<int> sol;
        sol.resize(dimension);

        sol[0] = depots[0];

        for (int i = 1, j = 1; i < dimension; i++)
            if (i != depots[0])
                sol[i] = j++;

        shuffle(sol.begin() + 1, sol.end(), rng);

        return sol;
    }
    vector<point> get_points_from_indices(const vector<int> &indices) const
    {
        vector<point> points;
        for (int i = 0; i < indices.size(); i++)
            points.push_back(nodes[indices[i]]);
        return points;
    }

    double calculate_cost(const vector<point> &sol)
    {
        double cost = 0.;
        for (int i = 1; i < sol.size(); i++)
            cost += dist(sol[i], sol[prev_node(i)]);

        cost += sqrt(pow(sol[0][0] - sol[sol.size() - 1][0], 2) +
                     pow(sol[0][1] - sol[sol.size() - 1][1], 2));

        return cost;
    }

    pair<vector<int>, set<int>> generate_non_consequtive_candidates(const vector<int> &sol, int nodes_to_remove, int candidate_spaces)
    {

        vector<int> new_sol;

        set<int> nodes_to_remove_set, candidate_spaces_set;

        while (1) // TODO: dp solver for edge case
        {
            candidate_spaces_set = generate_random_set(candidate_spaces, 0, sol.size() - 1);
            nodes_to_remove_set = generate_random_set(nodes_to_remove, 0, sol.size() - 1);
            new_sol.clear();

            bool f = 0;

            for (int i : sol)
            {
                if (nodes_to_remove_set.find(i) != nodes_to_remove_set.end() &&
                    candidate_spaces_set.find(i) != candidate_spaces_set.end())
                {
                    f = 1;
                    break;
                }

                if (nodes_to_remove_set.find(i) == nodes_to_remove_set.end())
                    new_sol.push_back(i);
                else
                    new_sol.push_back(-1);

                if (candidate_spaces_set.find(i) != candidate_spaces_set.end())
                    new_sol.push_back(-1);
            }

            for (int i = 0; i < new_sol.size(); i++)
                if (new_sol[i] == -1 && new_sol[prev_node(i, new_sol.size())] == -1)
                {
                    f = 1;
                    break;
                }

            if (!f)
                break;
        }

        return {new_sol, nodes_to_remove_set};
    }

    vector<int> iter_improve_random(const vector<int> &sol)
    {

        int nodes_to_remove = 10,
            candidate_spaces = 10;

        auto [candidated_vector, nodes_to_remove_set] = generate_non_consequtive_candidates(sol, nodes_to_remove, candidate_spaces);

        auto old_candidated_vector = candidated_vector; // za debug

        // for (int i : new_sol)
        //     cout << i << " ";
        // cout << endl;

        vector<vector<double>>
            cost_matrix(nodes_to_remove,
                        vector<double>(nodes_to_remove + candidate_spaces, 0.));

        assert(count(candidated_vector.begin(), candidated_vector.end(), -1) == nodes_to_remove + candidate_spaces);

        // for (int i : sol)
        //     cout << i << " ";
        // cout << endl;

        // for (int i : candidated_vector)
        //     cout << i << " ";
        // cout << endl;

        vector<double> no_place_vector(nodes_to_remove + candidate_spaces, 0.);

        int j = 0;
        for (int i : nodes_to_remove_set)
        {
            int l = 0;
            for (int k = 0; k < candidated_vector.size(); ++k)
                if (candidated_vector[k] == -1)
                {
                    int next = next_node(k, candidated_vector.size());
                    int prev = prev_node(k, candidated_vector.size());

                    point prev_point = nodes[candidated_vector[prev]];
                    point next_point = nodes[candidated_vector[next]];

                    // cout << "prev: " << candidated_vector[prev] << endl;
                    // cout << "i: " << i << endl;
                    // cout << "next: " << candidated_vector[next] << endl;

                    cost_matrix[j][l] = dist(prev_point, nodes[i]) + dist(nodes[i], next_point);

                    no_place_vector[l] = dist(prev_point, next_point);

                    // if (i == 3)
                    //     cout << "HMMMM " << no_add_cost << " " << add_cost << endl;

                    ++l;
                }
            ++j;
        }

        // for (int i = 0; i < cost_matrix.size(); ++i)
        // {
        //     for (int j = 0; j < cost_matrix[i].size(); ++j)
        //     {
        //         cout << cost_matrix[i][j] << " ";
        //     }
        //     cout << endl;
        // }

        vector<vector<bool>> result = lp_minimizer.minimize(cost_matrix, no_place_vector);

        map<int, int> m;
        for (int i = 0; i < result.size(); ++i)
        {
            for (int j = 0; j < result[i].size(); ++j)
            {
                if (result[i][j])
                {
                    m[j] = i;
                    break;
                }
            }
        }

        vector<int> nodes_to_remove_vector(nodes_to_remove_set.begin(), nodes_to_remove_set.end());

        for (int i = 0, j = 0; i < candidated_vector.size(); ++i)
        {
            if (candidated_vector[i] == -1)
            {
                if (m.find(j) != m.end())
                {
                    candidated_vector[i] = nodes_to_remove_vector[m[j]];
                }

                ++j;
            }
        }

        vector<int> new_sol;
        for (int i : candidated_vector)
            if (i != -1)
                new_sol.push_back(i);

        // cout << "Improvement: "
        //      << calculate_cost(get_points_from_indices(sol))
        //      << " -> "
        //      << calculate_cost(get_points_from_indices(new_sol)) << endl;

        if (!(calculate_cost(get_points_from_indices(sol)) + 0.1 > calculate_cost(get_points_from_indices(new_sol))))
        {
            cout << "no" << endl;

            cout << calculate_cost(get_points_from_indices(sol)) << " "
                 << calculate_cost(get_points_from_indices(new_sol)) << endl;

            cout
                << "=========" << endl;

            for (int i : old_candidated_vector)
                cout << i << endl;
            cout << endl;

            for (int i : sol)
                cout << i << " (" << nodes[i][0] << ", " << nodes[i][1] << ") " << endl;

            cout << "->" << endl;

            for (int i : new_sol)
                cout << i << " (" << nodes[i][0] << ", " << nodes[i][1] << ") " << endl;
            cout << endl;

            for (int i : nodes_to_remove_set)
                cout << i << " (" << nodes[i][0] << ", " << nodes[i][1] << ") " << endl;
            cout << endl;

            for (int i = 0; i < cost_matrix.size(); ++i)
            {
                for (int j = 0; j < cost_matrix[i].size(); ++j)
                {
                    cout << cost_matrix[i][j] << " ";
                }
                cout << endl;
            }
            for (int i = 0; i < no_place_vector.size(); ++i)
                cout << no_place_vector[i] << " ";
            cout << endl;

            for (int i = 0; i < result.size(); ++i)
            {
                for (int j = 0; j < result[i].size(); ++j)
                {
                    cout << result[i][j] << " ";
                }
                cout << endl;
            }

            cout << "=========" << endl;
        }

        // neki bug
        // assert(calculate_cost(get_points_from_indices(sol)) + 0.1 > calculate_cost(get_points_from_indices(new_sol)));

        return calculate_cost(get_points_from_indices(sol)) + 0.1 > calculate_cost(get_points_from_indices(new_sol)) ? new_sol : sol;

        // // for (int i : candidates_insert)
        // //     cout << i << " ";
        // // cout << endl;

        // // for (int i : candidates_replace)
        // //     cout << i << " ";
        // // cout << endl;

        // vector<vector<double>> cost_table(to_replace,
        //                                   vector<double>(to_replace + to_insert, 0));

        // // initially
        // // 1 0 0 0
        // // 0 1 0 0

        // // TODO: make into a function
        // for (int i = 0; i < to_replace; ++i)
        //     for (int j = 0; j < to_replace; ++j)
        //     {
        //         // insert i node into j node
        //         int candidate = candidates_replace[i]; // index in sol

        //         int prev = prev_node(candidate);
        //         int next = next_node(candidate);

        //         point prev_point = nodes[sol[prev]];
        //         point candidate_point = nodes[sol[candidate]];
        //         point next_point = nodes[sol[next]];

        //         int candidate_to_insert = candidates_replace[j];

        //         int prev_to_insert = prev_node(candidate_to_insert);
        //         int next_to_insert = next_node(candidate_to_insert);

        //         point prev_to_insert_point = nodes[sol[prev_to_insert]];
        //         point candidate_to_insert_point = nodes[sol[candidate_to_insert]];
        //         point next_to_insert_point = nodes[sol[next_to_insert]];

        //         // double cost = (dist(prev_point, next_point) +                          //   (d1 + d2')
        //         //                dist(prev_to_insert_point, candidate_to_insert_point) + //
        //         //                dist(candidate_to_insert_point, next_to_insert_point))  //
        //         //               - (dist(prev_point, candidate_point) +                   // - (d1' + d2)
        //         //                  dist(candidate_point, next_point) +
        //         //                  dist(prev_to_insert_point, next_to_insert_point));

        //         double prev_cost = dist(prev_point, candidate_point) +
        //                            dist(candidate_point, next_point) +
        //                            dist(prev_to_insert_point, candidate_to_insert_point) +
        //                            dist(candidate_to_insert_point, next_to_insert_point);

        //         double new_cost = dist(prev_point, candidate_to_insert_point) +
        //                           dist(candidate_to_insert_point, next_point) +
        //                           dist(prev_to_insert_point, candidate_point) +
        //                           dist(candidate_point, next_to_insert_point);

        //         cost_table[i][j] = new_cost - prev_cost;
        //     }

        // for (int i = 0; i < to_replace; ++i)
        //     for (int j = to_replace; j < to_replace + to_insert; ++j)
        //     {
        //         // insert i node in between j node and j + 1 node
        //         int candidate = candidates_replace[i]; // index in sol

        //         int prev = prev_node(candidate);
        //         int next = next_node(candidate);

        //         point prev_point = nodes[sol[prev]];
        //         point candidate_point = nodes[sol[candidate]];
        //         point next_point = nodes[sol[next]];

        //         int candidate_to_insert = candidates_insert[j - to_replace];

        //         int prev_to_insert = prev_node(candidate_to_insert);
        //         int next_to_insert = next_node(candidate_to_insert);

        //         point prev_to_insert_point = nodes[sol[prev_to_insert]];
        //         point candidate_to_insert_point = candidate_point;
        //         point next_to_insert_point = nodes[sol[next_to_insert]];

        //         double prev_cost = dist(prev_point, candidate_point) +
        //                            dist(candidate_point, next_point) +
        //                            dist(prev_to_insert_point, candidate_to_insert_point) +
        //                            dist(candidate_to_insert_point, next_to_insert_point);

        //         double new_cost = dist(prev_point, candidate_to_insert_point) +
        //                           dist(candidate_to_insert_point, next_point) +
        //                           dist(prev_to_insert_point, candidate_point) +
        //                           dist(candidate_point, next_to_insert_point);

        //         cost_table[i][j] = new_cost - prev_cost;
        //     }

        // // cout << endl;
        // // for (int i = 0; i < to_replace; ++i)
        // // {
        // //     for (int j = 0; j < to_replace + to_insert; ++j)
        // //         printf("%7.2f ", cost_table[i][j]);
        // //     cout << endl;
        // // }
        // // cout << endl;

        // vector<vector<bool>> result = lp_minimizer.minimize(cost_table);

        // // for (int i = 0; i < to_replace; ++i)
        // // {
        // //     for (int j = 0; j < to_replace + to_insert; ++j)
        // //         cout << result[i][j] << " ";
        // //     cout << endl;
        // // }

        // vector<int> new_sol = sol;

        // for (int i = 0; i < to_replace; ++i)
        //     for (int j = 0; j < to_replace; ++j)
        //         if (result[i][j])
        //             swap(new_sol[candidates_replace[i]],
        //                  new_sol[candidates_replace[j]]);

        // for (int i = 0; i < to_replace; ++i)
        //     for (int j = to_replace; j < to_replace + to_insert; ++j)
        //         if (result[i][j])
        //         {
        //             int candidate = candidates_insert[j - to_replace];

        //             auto pos = new_sol.insert(new_sol.begin() + candidate + 1,
        //                                       candidates_replace[i]);

        //             for (int k = 0; k < new_sol.size(); ++k)
        //                 if (new_sol.begin() + k != pos &&
        //                     new_sol[k] == candidates_replace[i])
        //                     new_sol.erase(new_sol.begin() + k);
        //         }

        // cout << new_sol.size() << endl;

        // cout << "New cost: " << calculate_cost(get_points_from_indices(sol)) << "->" << calculate_cost(get_points_from_indices(new_sol)) << endl;

        // return new_sol;
    }
} typedef problem;

// 0 4 1 2 3
// ->
// 0 1 4 3 2
//

int main(int argc, char *argv[])
{
    // freopen("test/custom.vrp", "r", stdin);

    // KDTree<point> kdtree;
    // kdtree.build({{0, 0}, {1, 1}});

    // auto r = kdtree.radiusSearch({0, 0}, 2);

    // for (auto i : r)
    //     cout << i << endl;

    // return 0;

    problem p;

    p.load_problem();
    // p.print_problem();

    // auto v = p.get_points_from_indices(p.random_solve());

    // for (auto [x, y] : v)
    //     cout << x << ", " << y << endl;

    auto solution = p.random_solve();

    for (int i = 0; i < 100; ++i)
    {
        solution = p.iter_improve_random(solution);
        // cout << i << endl;
    }

    // cout << "Cost: " << problem::calculate_cost(rand_sol) << endl;

    for (auto [x, y] : p.get_points_from_indices(solution))
        cout << x << ", " << y << endl;

    return 0;
}
