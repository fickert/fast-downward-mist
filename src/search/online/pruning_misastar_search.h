#ifndef ONLINE_PRUNING_MISASTAR_SEARCH_NAIVE_H
#define ONLINE_PRUNING_MISASTAR_SEARCH_NAIVE_H

#include "multiple_initial_states_astar_search.h"

namespace online {
class PruningMISAStarSearch : public MultipleInitialStateAStarSearch {
	auto prune_state(std::size_t reference_node_index, const GlobalState &state) const -> bool override;
	auto is_path_consistent(std::size_t reference_node_index, const GlobalState &state) const -> bool;

    std::size_t return_best_index(std::vector<std::unique_ptr<StateOpenList>> &open_lists) override;

public:
	PruningMISAStarSearch(const options::Options &opts,
	                      std::shared_ptr<StateRegistry> state_registry, 
	                      std::shared_ptr<AbstractTask> task,
	                      StateID initial_state_id,
	                      std::vector<std::pair<StateID, int>> &&reference_nodes,
	                      std::vector<std::pair<OperatorID, int>> &&planned_execution,
	                      int moving_average_size,
	                      double initial_avg_expansion_delay_value = 0.,
	                      int last_expansions = 0);
    virtual ~PruningMISAStarSearch() = default;
};
}

#endif
