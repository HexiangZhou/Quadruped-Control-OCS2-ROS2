#pragma once

#include "motion_control/legged_wbc/Task.h"

#include <memory>

// Hierarchical Optimization Quadratic Program
class HoQp {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using HoQpPtr = std::shared_ptr<HoQp>;

  explicit HoQp(const Task& task) : HoQp(task, nullptr){};

  HoQp(Task task, HoQpPtr higherProblem);

  ocs2::matrix_t getStackedZMatrix() const { return stackedZ_; }

  Task getStackedTasks() const { return stackedTasks_; }

  ocs2::vector_t getStackedSlackSolutions() const { return stackedSlackVars_; }

  ocs2::vector_t getSolutions() const {
    ocs2::vector_t x = xPrev_ + stackedZPrev_ * decisionVarsSolutions_;
    return x;
  }

  size_t getSlackedNumVars() const { return stackedTasks_.d_.rows(); }

 private:
  void initVars();
  void formulateProblem();
  void solveProblem();

  void buildHMatrix();
  void buildCVector();
  void buildDMatrix();
  void buildFVector();

  void buildZMatrix();
  void stackSlackSolutions();

  Task task_, stackedTasksPrev_, stackedTasks_;
  HoQpPtr higherProblem_;

  bool hasEqConstraints_{}, hasIneqConstraints_{};
  size_t numSlackVars_{}, numDecisionVars_{};
  ocs2::matrix_t stackedZPrev_, stackedZ_;
  ocs2::vector_t stackedSlackSolutionsPrev_, xPrev_;
  size_t numPrevSlackVars_{};

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_, d_;
  ocs2::vector_t c_, f_;
  ocs2::vector_t stackedSlackVars_, slackVarsSolutions_, decisionVarsSolutions_;

  // Convenience matrices that are used multiple times
  ocs2::matrix_t eyeNvNv_;
  ocs2::matrix_t zeroNvNx_;
};

