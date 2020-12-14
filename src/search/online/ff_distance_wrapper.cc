#include "ff_distance_wrapper.h"

#include "../evaluation_context.h"

namespace online {
FFDistanceWrapper::FFDistanceWrapper(std::shared_ptr<ff_heuristic::FFHeuristic> ff_heuristic)
	: ff_heuristic(ff_heuristic), cache(UNINITIALIZED) {}

auto FFDistanceWrapper::compute_result(EvaluationContext &eval_context) -> EvaluationResult {
    auto result = EvaluationResult();
	auto &cache_entry = cache[eval_context.get_state()];
	if (cache_entry == UNINITIALIZED) {
#ifndef NDEBUG
		assert(ff_heuristic->last_evaluated == eval_context.get_state().get_id());
#endif
		cache_entry = ff_heuristic->last_relaxed_plan_length;
	}
	result.set_evaluator_value(cache_entry);
	return result;
}
}
