#ifndef ONLINE_FF_DISTANCE_WRAPPER_H
#define ONLINE_FF_DISTANCE_WRAPPER_H

#include "../heuristics/ff_heuristic.h"

namespace online {
class FFDistanceWrapper : public Evaluator {
    std::shared_ptr<ff_heuristic::FFHeuristic> ff_heuristic;
	PerStateInformation<int> cache;

	static constexpr auto UNINITIALIZED = -2;

public:
	explicit FFDistanceWrapper(std::shared_ptr<ff_heuristic::FFHeuristic> ff_heuristic);
    ~FFDistanceWrapper() override {};

    auto compute_result(EvaluationContext &eval_context) -> EvaluationResult override;
    void get_path_dependent_evaluators(std::set<Evaluator *> &) override {};
};
}

#endif
