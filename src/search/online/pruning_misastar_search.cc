#include "pruning_misastar_search.h"

#include "time_spent_evaluator.h"
#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../evaluators/const_evaluator.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/max_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../evaluators/weighted_evaluator.h"
#include "../open_list_factory.h"
#include "../option_parser.h"
#include "../pruning_method.h"

#include "../algorithms/ordered_set.h"
#include "../task_utils/successor_generator.h"
#include "../open_lists/tiebreaking_open_list.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <set>
#include "../task_utils/task_properties.h"

using namespace std;

namespace online {
PruningMISAStarSearch::PruningMISAStarSearch(const options::Options &opts, std::shared_ptr<StateRegistry> state_registry, std::shared_ptr<AbstractTask> task, StateID initial_state_id,
	std::vector<std::pair<StateID, int>> &&reference_nodes, std::vector<std::pair<OperatorID, int>> &&planned_execution, int moving_average_size, double initial_avg_expansion_delay_value, int last_expansions)
    : MultipleInitialStateAStarSearch(opts, state_registry, task, initial_state_id, std::move(reference_nodes), std::move(planned_execution), moving_average_size, initial_avg_expansion_delay_value, last_expansions) {}

auto PruningMISAStarSearch::prune_state(std::size_t reference_node_index, const GlobalState &state) const -> bool {
	const auto &[reference_state_id, arrival_time] = reference_nodes[reference_node_index];
	return (statistics.get_expanded() + 1) >= arrival_time && !is_path_consistent(reference_node_index, state);
}

auto PruningMISAStarSearch::is_path_consistent(std::size_t reference_node_index, const GlobalState &state) const -> bool {
	// Note: this check could probably be implemented a bit more efficiently by not always comparing the full path
	auto plan = Plan();
	search_spaces[reference_node_index]->trace_path(state, plan);
	auto plan_it = std::cbegin(plan);
	for (auto execution_it = next_action[reference_node_index];
	     execution_it != std::cend(planned_execution) && plan_it != std::cend(plan) && execution_it->second <= (statistics.get_expanded() + 1);
	     ++execution_it, ++plan_it) {
		if (execution_it->first != *plan_it)
			return false;
	}
	return true;
}

std::size_t PruningMISAStarSearch::return_best_index(vector<unique_ptr<StateOpenList>> &open_lists) {
	// make sure the node at the front of each open list is consistent with the planned execution
	for (auto i = 0u; i < open_lists.size(); ++i) {
		auto &open_list = *open_lists[i];
		const auto &[reference_state_id, arrival_time] = reference_nodes[i];

		if (statistics.get_expanded() < arrival_time)
			// reference node lies in the future ==> all nodes are still consistent
			continue;

		while (!open_list.empty()) {
			const auto state = state_registry->lookup_state(open_list.top());
			if (is_path_consistent(i, state))
				break;
			open_list.remove_min();
		}
	}

	std::vector<std::vector<int>> open_list_keys;
	for (const auto &open_list : open_lists)
		open_list_keys.emplace_back(open_list->empty() ? std::vector<int>{} : open_list->get_min_keys());

	assert(reference_nodes.size() == open_lists.size());
	for (auto i = 0u; i < open_list_keys.size(); ++i) {
		auto &open_list_key = open_list_keys[i];
		if (open_list_key.empty())
			continue;
		auto &reference_node = reference_nodes[i];
		assert(open_list_key.front() >= 0);
		open_list_key.front() = reference_node.second + open_list_key.front() * task->get_execution_time();
		// the following depends on the open list being a tie breaking open list, where the second key is the heuristic value
		// ideally, we would retrieve the top state, and compute/lookup the heuristic value from a corresponding EvaluationContext
		assert(open_list_key.size() == 2u);
		const auto estimated_total_planning_time = statistics.get_expanded() + std::lround(min_distance[i] * expansion_delay->get_avg_expansion_delay());
		if (estimated_total_planning_time > reference_node.second)
			open_list_key.front() = std::numeric_limits<int>::max();
		// use the index as the third tie breaker:
		// prefer search nodes corresponding to reference nodes that are reached later, as planning is more likely to finish before reaching these
		open_list_key.push_back(open_list_keys.size() - i);
	}

	return std::distance(std::begin(open_list_keys), std::min_element(std::begin(open_list_keys), std::end(open_list_keys), [](const auto &lhs, const auto &rhs) {
		if (rhs.empty())
			return !lhs.empty();
		if (lhs.empty())
			return false;
		return std::lexicographical_compare(std::begin(lhs), std::end(lhs), std::begin(rhs), std::end(rhs));
	}));
}

}
