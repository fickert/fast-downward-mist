#ifndef EVALUATORS_MAX_EVALUATOR_H
#define EVALUATORS_MAX_EVALUATOR_H

#include "combining_evaluator.h"

#include <vector>

namespace options {
class Options;
}

namespace max_evaluator {
class MaxEvaluator : public combining_evaluator::CombiningEvaluator {
protected:
    virtual int combine_values(const std::vector<int> &values) override;

public:
    explicit MaxEvaluator(const options::Options &opts);
    explicit MaxEvaluator(const std::vector<std::shared_ptr<Evaluator>> &evals);
    virtual ~MaxEvaluator() override;
};

class RelaxedMaxEvaluator : public combining_evaluator::CombiningEvaluator {
protected:
	virtual int combine_values(const std::vector<int> &values) override;

public:
	explicit RelaxedMaxEvaluator(const options::Options &opts);
	explicit RelaxedMaxEvaluator(const std::vector<std::shared_ptr<Evaluator>> &evals);
	virtual ~RelaxedMaxEvaluator() override;
};
}

#endif