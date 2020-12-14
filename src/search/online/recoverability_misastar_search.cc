#include "recoverability_misastar_search.h"

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
RecoverabilityMISAStarSearch::RecoverabilityMISAStarSearch(const options::Options &opts,
                                                           std::shared_ptr<StateRegistry> state_registry,
                                                           std::shared_ptr<AbstractTask> task,
                                                           StateID initial_state_id,
                                                           std::vector<std::pair<StateID, int>> &&reference_nodes,
                                                           std::vector<std::pair<OperatorID, int>> &&planned_execution,
                                                           int moving_average_size,
                                                           double initial_avg_expansion_delay_value,
                                                           int last_expansions)
	: MultipleInitialStateAStarSearch(opts, state_registry, task, initial_state_id, std::move(reference_nodes), std::move(planned_execution), moving_average_size, initial_avg_expansion_delay_value, last_expansions),
	  end_time(planned_execution.empty() ? 0 : planned_execution.back().second + task->get_operator_cost(planned_execution.back().first.get_index(), false) * task->get_execution_time()) {}

std::size_t RecoverabilityMISAStarSearch::return_best_index(vector<unique_ptr<StateOpenList>> &open_lists) {
	std::vector<std::vector<int>> open_list_keys;
	for (const auto &open_list : open_lists)
		open_list_keys.emplace_back(open_list->empty() ? std::vector<int>{} : open_list->get_min_keys());
	assert(reference_nodes.size() == open_lists.size());
	for (auto i = 0u; i < open_list_keys.size(); ++i) {
		auto &open_list_key = open_list_keys[i];
		if (open_list_key.empty())
			continue;
		const auto &reference_node = reference_nodes[i];
		assert(open_list_key.front() >= 0);
		open_list_key.front() = reference_node.second + open_list_key.front() * task->get_execution_time();
		// the following depends on the open list being a tie breaking open list, where the second key is the heuristic value
		// ideally, we would retrieve the top state, and compute/lookup the heuristic value from a corresponding EvaluationContext
		assert(open_list_key.size() == 2u);
		const auto estimated_total_planning_time = statistics.get_expanded() + std::lround(min_distance[i] * expansion_delay->get_avg_expansion_delay());
		auto [overshot, last_action_it] = get_overshot(i, estimated_total_planning_time);
		const auto last_node_overshot = std::max(estimated_total_planning_time - reference_nodes.back().second, 0l);
		assert(overshot >= last_node_overshot);
		// reduce overshot by execution time that does not need to be reversed because the paths are consistent
		if (overshot > 0 && next_action[i] != last_action_it) {
			auto plan = Plan();
			search_spaces[i]->trace_path(state_registry->lookup_state(open_lists[i]->top()), plan);
			auto plan_it = std::cbegin(plan);
			for (auto execution_it = next_action[i]; execution_it != last_action_it &&
			                                         plan_it != std::cend(plan) &&
			                                         execution_it->first == *plan_it; ++execution_it, ++plan_it) {
				overshot -= task->get_operator_cost(plan_it->get_index(), false) * task->get_execution_time();
			}
		}
		assert(overshot >= 0);
		open_list_key.front() += 2 * overshot - last_node_overshot;
	}

	return std::distance(std::begin(open_list_keys), std::min_element(std::begin(open_list_keys), std::end(open_list_keys), [](const auto &lhs, const auto &rhs) {
		if (rhs.empty())
			return true;
		if (lhs.empty())
			return false;
		return std::lexicographical_compare(std::begin(lhs), std::end(lhs), std::begin(rhs), std::end(rhs));
	}));
}

auto RecoverabilityMISAStarSearch::get_overshot(std::size_t reference_node_index, long estimated_total_planning_time) const -> std::pair<long, decltype(planned_execution)::const_iterator> {
	const auto &reference_node = reference_nodes[reference_node_index];
	if (estimated_total_planning_time < reference_node.second)
		return { 0, std::end(planned_execution) };
	const auto planned_execution_it = std::find_if(next_action[reference_node_index], std::cend(planned_execution), [estimated_total_planning_time](const auto &planned_execution_step) {
		return planned_execution_step.second >= estimated_total_planning_time;
	});
	return { (planned_execution_it != std::cend(planned_execution) ? planned_execution_it->second : std::max<long>(end_time, estimated_total_planning_time)) - reference_node.second,
	         planned_execution_it };
}

}
