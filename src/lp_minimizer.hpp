#include <vector>
#include "gurobi_c++.h"

using namespace std;

#ifndef __LP_MINIMIZER_H__
#define __LP_MINIMIZER_H__

class LPMinimizer
{
private:
    GRBEnv env;

public:
    LPMinimizer() : env(GRBEnv())
    {
        env.set(GRB_IntParam_OutputFlag, 0);
    }

    vector<vector<bool>> minimize(const vector<vector<double>> &cost_table, const vector<double> &no_place_vector)
    {
        GRBModel model = GRBModel(env);
        model.set(GRB_StringAttr_ModelName, "relaxation");

        auto table = model.addVars(cost_table[0].size() * cost_table.size(),
                                   GRB_BINARY);

        for (int i = 0; i < cost_table.size(); ++i)
        {
            GRBLinExpr sum = 0;

            for (int j = 0; j < cost_table[i].size(); ++j)
                sum += table[i * cost_table[i].size() + j];

            model.addConstr(sum == 1);
        }

        auto no_place_vector_table = model.addVars(no_place_vector.size(), GRB_BINARY);

        for (int j = 0; j < cost_table[0].size(); ++j)
        {
            GRBLinExpr sum = 0;

            for (int i = 0; i < cost_table.size(); ++i)
                sum += table[i * cost_table[i].size() + j];

            sum += no_place_vector_table[j];

            model.addConstr(sum == 1);
        }

        GRBLinExpr obj = 0;

        for (int i = 0; i < cost_table.size(); ++i)
            for (int j = 0; j < cost_table[i].size(); ++j)
                obj += table[i * cost_table[i].size() + j] * cost_table[i][j];

        for (int j = 0; j < no_place_vector.size(); ++j)
            obj += no_place_vector_table[j] * no_place_vector[j];

        model.setObjective(obj, GRB_MINIMIZE);
        model.optimize();

        vector<vector<bool>> solution_table(cost_table.size(),
                                            vector<bool>(cost_table[0].size(),
                                                         false));

        for (int i = 0; i < solution_table.size(); ++i)
            for (int j = 0; j < solution_table[i].size(); ++j)
                solution_table[i][j] = table[i * cost_table[i].size() + j].get(GRB_DoubleAttr_X) > 0.5;

        // for (int i = 0; i < cost_table.size(); ++i)
        // {
        //     for (int j = 0; j < cost_table[i].size(); ++j)
        //     {
        //         cout << (table[i * cost_table[i].size() + j].get(GRB_DoubleAttr_X) > 0.5
        //                      ? "1"
        //                      : "0")
        //              << " ";
        //     }
        //     cout << endl;
        // }

        return solution_table;
    }
};

#endif // __LP_MINIMIZER_H__