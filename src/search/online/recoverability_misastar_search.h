#ifndef ONLINE_RECOVERABILITY_MISASTAR_SEARCH_H
#define ONLINE_RECOVERABILITY_MISASTAR_SEARCH_H

#include "multiple_initial_states_astar_search.h"

namespace online {
class RecoverabilityMISAStarSearch : public MultipleInitialStateAStarSearch {
	// end of planned execution
	const int end_time;

	auto return_best_index(std::vector<std::unique_ptr<StateOpenList>> &open_lists) -> std::size_t override;
	auto get_overshot(std::size_t reference_node_index, long estimated_total_planning_time) const -> std::pair<long, decltype(planned_execution)::const_iterator>;

public:
	RecoverabilityMISAStarSearch(const options::Options &opts,
	                             std::shared_ptr<StateRegistry> state_registry, 
	                             std::shared_ptr<AbstractTask> task,
	                             StateID initial_state_id,
	                             std::vector<std::pair<StateID, int>> &&reference_nodes,
	                             std::vector<std::pair<OperatorID, int>> &&planned_execution,
	                             int moving_average_size,
	                             double initial_avg_expansion_delay_value = 0.,
	                             int last_expansions = 0);
    virtual ~RecoverabilityMISAStarSearch() = default;
};
}

#endif
