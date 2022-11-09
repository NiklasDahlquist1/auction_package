
#ifndef OPTIMIZATION_ORTOOLS_H
#define OPTIMIZATION_ORTOOLS_H

#include <vector>

#include "ortools/base/logging.h"
#include "ortools/linear_solver/linear_solver.h"


#include "optimization_tester.hpp"
#include "optimization_ortools.h"




namespace operations_research
{



  // task matching, assigns the maximum number of tasks at the minimum cost
  // cost[i][j], i=worker, j=cost associated with task j
  // returns array result[i][k] with k tasks associated with each worker i.  empty if no tasks             ////   if -1 that worker has no task assigned
  // rewards[j], reward associated with finishing task j
  // maxNumOfTasksAssigned: the number of tasks that can be assigned to a single agent
  std::vector<std::vector<double>> taskMatching(const std::vector<std::vector<double>> costs, const std::vector<double>& rewards, const int maxNumOfTasksAssigned, const std::string solver_name)
  {
    const int num_workers = costs.size();
    const int num_tasks = costs[0].size();

    //std::cout << "workers: " << num_workers << " tasks: " << num_tasks << std::endl;


    // Create solver with
    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver(solver_name)); // SCIP CP-SAT, GPLK, GLOP, (GUROBI), CLP, CBC
    if (!solver)
    {
      std::cout << "ERROR: could not create solver" << std::endl; //works for now
    }

    // Variables
    // x[i][j] is an array of 0-1 variables, which will be 1
    // if worker i is assigned to task j.
    std::vector<std::vector<const MPVariable *>> x(num_workers, std::vector<const MPVariable *>(num_tasks));

    for (int i = 0; i < num_workers; ++i)
    {
      for (int j = 0; j < num_tasks; ++j)
      {
        if(costs[i][j] > 0)
        {
          x[i][j] = solver->MakeBoolVar(""); //->MakeIntVar(0, 1, "");
        }
      }
    }



    for(int i = 0; i < num_workers; ++i)
    {
      LinearExpr sum_j;
      for(int j = 0; j < num_tasks; ++j)
      {
        if(costs[i][j] > 0)
        {
          sum_j += x[i][j];
        }
      }
      solver->MakeRowConstraint(sum_j <= maxNumOfTasksAssigned); // change here to assign multiple tasks to an agent
    }

    for(int j = 0; j < num_tasks; ++j)
    {
      LinearExpr sum_i;
      for(int i = 0; i < num_workers; ++i)
      {
        if(costs[i][j] > 0)
        {
          sum_i += x[i][j];
        }
      }
      solver->MakeRowConstraint(sum_i <= 1);
    }


    //find max cost
    double maxCost = -1;
    for(int i = 0; i < num_workers; ++i)
    {
      for(int j = 0; j < num_tasks; ++j)
      {
        if(costs[i][j] > maxCost)
        {
          maxCost = costs[i][j];
        }
      }
    }    



    std::vector<std::vector<double>> profitsFromCosts(num_workers);
    for(int i = 0; i < num_workers; ++i)
    {
      profitsFromCosts[i].resize(num_tasks);
    }


    //convert costs to "profits" // this will mess up the original cost matrix, but we dont care about that for now
    for(int i = 0; i < num_workers; ++i)
    {
      for(int j = 0; j < num_tasks; ++j)
      {
        if(costs[i][j] > 0)
        {
          profitsFromCosts[i][j] = -costs[i][j] + maxCost + 1;
        }
        else
        {
          profitsFromCosts[i][j] = -1;
        }
        //costs[i][j] = 1 / costs[i][j];//costs[i][j] = -costs[i][j];
      }
    }



    // Objective.
    MPObjective *const objective = solver->MutableObjective();
    for (int i = 0; i < num_workers; ++i)
    {
      for (int j = 0; j < num_tasks; ++j)
      {
        if(profitsFromCosts[i][j] > 0)
        {
          objective->SetCoefficient(x[i][j], profitsFromCosts[i][j] + rewards[j]);
        }
      }
    }
    objective->SetMaximization();


    

    auto start = std::chrono::high_resolution_clock::now();

    // Solve
    const MPSolver::ResultStatus result_status = solver->Solve();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end-start;
    //std::cout << "Pure solving time: " << elapsed.count() << " ms, result status: " << result_status << ", objective value: " << objective->Value() << "\n";


    // return associated task with each worker
    std::vector<std::vector<double>> result(num_workers);
    //initialize result
    
    bool assigned = false; // track if current worker (i) has been associated with a task

    for (int i = 0; i < num_workers; ++i)
    {
      //result[i].resize(1);

      assigned = false;
      int taskNumber = 0;
      for (int j = 0; j < num_tasks; ++j)
      {
        if(costs[i][j] > 0)
        {
          // Test if x[i][j] is 0 or 1 (with tolerance for floating point arithmetic).
          if (x[i][j]->solution_value() > 0.5)
          {
            //LOG(INFO) << "Worker " << i << " assigned to task " << j << ".  Cost = " << costs[i][j];
            result[i].push_back(j);
            //result[i][taskNumber] = j;
            taskNumber += 1;
            assigned = true;
          }
        }

      }

      if (assigned == false)
      {
        //result[i][taskNumber] = -1; //indicate that this worker has not been associated with a task
      }
    }
    //std::cout << "ehj" << std::endl;

    return result;
  }





















 // SPECIAL CONSTRAINTS FOR STATION

  // constraints[i][j]. i=station, j=task_index. constrain so that: sum_j( sum_i(x[i][j]) ) < max_tasks_per_group (for all j in constraints)    for each i


  // task matching, assigns the maximum number of tasks at the minimum cost
  // cost[i][j], i=worker, j=cost associated with task j
  // returns array result[i][k] with k tasks associated with each worker i.  empty if no tasks             ////   if -1 that worker has no task assigned
  // rewards[j], reward associated with finishing task j
  // maxNumOfTasksAssigned: the number of tasks that can be assigned to a single agent
  std::vector<std::vector<double>> taskMatching(const std::vector<std::vector<double>> costs, const std::vector<double>& rewards, const int maxNumOfTasksAssigned, const std::string solver_name, const std::vector<std::vector<int>> constraints_groups, int max_tasks_per_group)
  {
    const int num_workers = costs.size();
    const int num_tasks = costs[0].size();

    //std::cout << "workers: " << num_workers << " tasks: " << num_tasks << std::endl;


    // Create solver with
    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver(solver_name)); // SCIP CP-SAT, GPLK, GLOP, (GUROBI), CLP, CBC
    if (!solver)
    {
      std::cout << "ERROR: could not create solver" << std::endl; //works for now
    }

    // Variables
    // x[i][j] is an array of 0-1 variables, which will be 1
    // if worker i is assigned to task j.
    std::vector<std::vector<const MPVariable *>> x(num_workers, std::vector<const MPVariable *>(num_tasks));

    for (int i = 0; i < num_workers; ++i)
    {
      for (int j = 0; j < num_tasks; ++j)
      {
        if(costs[i][j] > 0)
        {
          x[i][j] = solver->MakeBoolVar(""); //->MakeIntVar(0, 1, "");
        }
      }
    }



    for(int i = 0; i < num_workers; ++i)
    {
      LinearExpr sum_j;
      for(int j = 0; j < num_tasks; ++j)
      {
        if(costs[i][j] > 0)
        {
          sum_j += x[i][j];
        }
      }
      solver->MakeRowConstraint(sum_j <= maxNumOfTasksAssigned); // change here to assign multiple tasks to an agent
    }

    for(int j = 0; j < num_tasks; ++j)
    {
      LinearExpr sum_i;
      for(int i = 0; i < num_workers; ++i)
      {
        if(costs[i][j] > 0)
        {
          sum_i += x[i][j];
        }
      }
      solver->MakeRowConstraint(sum_i <= 1);
    }


    //find max cost
    double maxCost = -1;
    for(int i = 0; i < num_workers; ++i)
    {
      for(int j = 0; j < num_tasks; ++j)
      {
        if(costs[i][j] > maxCost)
        {
          maxCost = costs[i][j];
        }
      }
    }    


    // add the constraints
    //std::cout << "NUMBER OF GROUPS: " << constraints_groups.size() << "\n";
    for(int c_s = 0; c_s < constraints_groups.size(); ++c_s)
    {
      LinearExpr sum_constr;
      for(int c_j = 0; c_j < constraints_groups[c_s].size(); ++c_j)
      {
        for(int c_i = 0; c_i < num_workers; ++c_i)
        {
          if(costs[c_i][constraints_groups[c_s][c_j]] > 0) // check if worker i is bidding for task c_j
          {
            sum_constr += x[c_i][constraints_groups[c_s][c_j]];
          }
        }
        //std::cout << "TASK NUMBER: " << constraints_groups[c_s][c_j] << "\n";
      }
      solver->MakeRowConstraint(sum_constr <= max_tasks_per_group);
    }





    std::vector<std::vector<double>> profitsFromCosts(num_workers);
    for(int i = 0; i < num_workers; ++i)
    {
      profitsFromCosts[i].resize(num_tasks);
    }


    //convert costs to "profits" // this will mess up the original cost matrix, but we dont care about that for now
    for(int i = 0; i < num_workers; ++i)
    {
      for(int j = 0; j < num_tasks; ++j)
      {
        if(costs[i][j] > 0)
        {
          profitsFromCosts[i][j] = -costs[i][j] + maxCost + 1;
        }
        else
        {
          profitsFromCosts[i][j] = -1;
        }
        //costs[i][j] = 1 / costs[i][j];//costs[i][j] = -costs[i][j];
      }
    }



    // Objective.
    MPObjective *const objective = solver->MutableObjective();
    for (int i = 0; i < num_workers; ++i)
    {
      for (int j = 0; j < num_tasks; ++j)
      {
        if(profitsFromCosts[i][j] > 0)
        {
          objective->SetCoefficient(x[i][j], profitsFromCosts[i][j] + rewards[j]);
        }
      }
    }
    objective->SetMaximization();


    

    auto start = std::chrono::high_resolution_clock::now();

    // Solve
    const MPSolver::ResultStatus result_status = solver->Solve();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end-start;
    //std::cout << "Pure solving time: " << elapsed.count() << " ms, result status: " << result_status << ", objective value: " << objective->Value() << "\n";


    // return associated task with each worker
    std::vector<std::vector<double>> result(num_workers);
    //initialize result
    
    bool assigned = false; // track if current worker (i) has been associated with a task

    for (int i = 0; i < num_workers; ++i)
    {
      //result[i].resize(1);

      assigned = false;
      int taskNumber = 0;
      for (int j = 0; j < num_tasks; ++j)
      {
        if(costs[i][j] > 0)
        {
          // Test if x[i][j] is 0 or 1 (with tolerance for floating point arithmetic).
          if (x[i][j]->solution_value() > 0.5)
          {
            //LOG(INFO) << "Worker " << i << " assigned to task " << j << ".  Cost = " << costs[i][j];
            result[i].push_back(j);
            //result[i][taskNumber] = j;
            taskNumber += 1;
            assigned = true;
          }
        }

      }

      if (assigned == false)
      {
        //result[i][taskNumber] = -1; //indicate that this worker has not been associated with a task
      }
    }
    //std::cout << "ehj" << std::endl;

    return result;
  }



} // namespace operations_research




#endif






