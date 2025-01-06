#include "motion_control/legged_wbc/HoQp.h"

#include <qpOASES.hpp>
#include <utility>


HoQp::HoQp(Task task, HoQp::HoQpPtr higherProblem) : task_(std::move(task)), higherProblem_(std::move(higherProblem)) {
  initVars();
  formulateProblem();
  solveProblem();
  // For next problem
  buildZMatrix();
  stackSlackSolutions();
}

void HoQp::initVars() {
  // Task variables
  numSlackVars_ = task_.d_.rows();
  hasEqConstraints_ = task_.a_.rows() > 0;
  hasIneqConstraints_ = numSlackVars_ > 0;

  // Pre-Task variables
  if (higherProblem_ != nullptr) {
    stackedZPrev_ = higherProblem_->getStackedZMatrix();
    stackedTasksPrev_ = higherProblem_->getStackedTasks();
    stackedSlackSolutionsPrev_ = higherProblem_->getStackedSlackSolutions();
    xPrev_ = higherProblem_->getSolutions();
    numPrevSlackVars_ = higherProblem_->getSlackedNumVars();

    numDecisionVars_ = stackedZPrev_.cols();
  } else {
    numDecisionVars_ = std::max(task_.a_.cols(), task_.d_.cols());

    stackedTasksPrev_ = Task(numDecisionVars_);
    stackedZPrev_ = ocs2::matrix_t::Identity(numDecisionVars_, numDecisionVars_);
    stackedSlackSolutionsPrev_ = Eigen::VectorXd::Zero(0);
    xPrev_ = Eigen::VectorXd::Zero(numDecisionVars_);
    numPrevSlackVars_ = 0;
  }

  stackedTasks_ = task_ + stackedTasksPrev_;

  // Init convenience matrices
  eyeNvNv_ = ocs2::matrix_t::Identity(numSlackVars_, numSlackVars_);
  zeroNvNx_ = ocs2::matrix_t::Zero(numSlackVars_, numDecisionVars_);
}

void HoQp::formulateProblem() {
  buildHMatrix();
  buildCVector();
  buildDMatrix();
  buildFVector();
}

void HoQp::buildHMatrix() {
  ocs2::matrix_t zTaTaz(numDecisionVars_, numDecisionVars_);

  if (hasEqConstraints_) {
    // Make sure that all eigenvalues of A_t_A are non-negative, which could arise due to numerical issues
    ocs2::matrix_t aCurrZPrev = task_.a_ * stackedZPrev_;
    zTaTaz = aCurrZPrev.transpose() * aCurrZPrev + 1e-12 * ocs2::matrix_t::Identity(numDecisionVars_, numDecisionVars_);
    // This way of splitting up the multiplication is about twice as fast as multiplying 4 matrices
  } else {
    zTaTaz.setZero();
  }

  h_ = (ocs2::matrix_t(numDecisionVars_ + numSlackVars_, numDecisionVars_ + numSlackVars_)  // clang-format off
            << zTaTaz, zeroNvNx_.transpose(),
                zeroNvNx_, eyeNvNv_)  // clang-format on
           .finished();
}

void HoQp::buildCVector() {
  ocs2::vector_t c = ocs2::vector_t::Zero(numDecisionVars_ + numSlackVars_);
  ocs2::vector_t zeroVec = ocs2::vector_t::Zero(numSlackVars_);

  ocs2::vector_t temp(numDecisionVars_);
  if (hasEqConstraints_) {
    temp = (task_.a_ * stackedZPrev_).transpose() * (task_.a_ * xPrev_ - task_.b_);
  } else {
    temp.setZero();
  }

  c_ = (ocs2::vector_t(numDecisionVars_ + numSlackVars_) << temp, zeroVec).finished();
}

void HoQp::buildDMatrix() {
  ocs2::matrix_t stackedZero = ocs2::matrix_t::Zero(numPrevSlackVars_, numSlackVars_);

  ocs2::matrix_t dCurrZ;
  if (hasIneqConstraints_) {
    dCurrZ = task_.d_ * stackedZPrev_;
  } else {
    dCurrZ = ocs2::matrix_t::Zero(0, numDecisionVars_);
  }

  // NOTE: This is upside down compared to the paper,
  // but more consistent with the rest of the algorithm
  d_ = (ocs2::matrix_t(2 * numSlackVars_ + numPrevSlackVars_, numDecisionVars_ + numSlackVars_)  // clang-format off
            << zeroNvNx_, -eyeNvNv_,
                stackedTasksPrev_.d_ * stackedZPrev_, stackedZero,
                dCurrZ, -eyeNvNv_)  // clang-format on
           .finished();
}

void HoQp::buildFVector() {
  ocs2::vector_t zeroVec = ocs2::vector_t::Zero(numSlackVars_);

  ocs2::vector_t fMinusDXPrev;
  if (hasIneqConstraints_) {
    fMinusDXPrev = task_.f_ - task_.d_ * xPrev_;
  } else {
    fMinusDXPrev = ocs2::vector_t::Zero(0);
  }

  f_ = (ocs2::vector_t(2 * numSlackVars_ + numPrevSlackVars_) << zeroVec,
        stackedTasksPrev_.f_ - stackedTasksPrev_.d_ * xPrev_ + stackedSlackSolutionsPrev_, fMinusDXPrev)
           .finished();
}

void HoQp::buildZMatrix() {
  if (hasEqConstraints_) {
    assert((task_.a_.cols() > 0));
    stackedZ_ = stackedZPrev_ * (task_.a_ * stackedZPrev_).fullPivLu().kernel();
  } else {
    stackedZ_ = stackedZPrev_;
  }
}

void HoQp::solveProblem() {
  auto qpProblem = qpOASES::QProblem(numDecisionVars_ + numSlackVars_, f_.size());
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  qpProblem.setOptions(options);
  int nWsr = 20;

  qpProblem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, nullptr, f_.data(), nWsr);
  ocs2::vector_t qpSol(numDecisionVars_ + numSlackVars_);

  qpProblem.getPrimalSolution(qpSol.data());

  decisionVarsSolutions_ = qpSol.head(numDecisionVars_);
  slackVarsSolutions_ = qpSol.tail(numSlackVars_);
}

void HoQp::stackSlackSolutions() {
  if (higherProblem_ != nullptr) {
    stackedSlackVars_ = Task::concatenateVectors(higherProblem_->getStackedSlackSolutions(), slackVarsSolutions_);
  } else {
    stackedSlackVars_ = slackVarsSolutions_;
  }
}

