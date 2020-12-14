#include "time_spent_evaluator.h"

#include "../search_statistics.h"

namespace online {
TimeSpentEvaluator::TimeSpentEvaluator(const SearchStatistics &statistics)
	: Evaluator(),
	  statistics(statistics) {}

auto TimeSpentEvaluator::compute_result(EvaluationContext &) -> EvaluationResult {
	auto result = EvaluationResult();
	result.set_evaluator_value(statistics.get_expanded());
	return result;
}

}
