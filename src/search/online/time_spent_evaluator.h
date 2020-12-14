#ifndef ONLINE_TIME_SPENT_EVALUATOR_H
#define ONLINE_TIME_SPENT_EVALUATOR_H

#include "../evaluator.h"

class SearchStatistics;

namespace online {
class TimeSpentEvaluator : public Evaluator {
protected:
	const SearchStatistics &statistics;

public:
	TimeSpentEvaluator(const SearchStatistics &statistics);
	~TimeSpentEvaluator() override = default;

	auto compute_result(EvaluationContext &) -> EvaluationResult override;
	void get_path_dependent_evaluators(std::set<Evaluator *> &) override {}
};
}

#endif
