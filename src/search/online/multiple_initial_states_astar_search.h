#ifndef ONLINE_MULTIPLE_INITIAL_STATES_ASTAR_SEARCH_H
#define ONLINE_MULTIPLE_INITIAL_STATES_ASTAR_SEARCH_H

#include "../open_list.h"
#include "../search_engine.h"

#include <cmath>
#include <memory>
#include <vector>
#include <deque>

class Evaluator;
class PruningMethod;
class OpenListFactory;

namespace options {
class Options;
}

namespace online {
class MultipleInitialStateAStarSearch : public SearchEngine {
protected:
	const bool reopen_closed_nodes;

	const std::vector<std::pair<StateID, int>> reference_nodes;
	const std::unordered_map<StateID, int> reference_nodes_lookup;
	static auto initialize_reference_nodes_lookup(const std::vector<std::pair<StateID, int>> &reference_nodes) -> std::unordered_map<StateID, int>;

	const std::vector<std::pair<OperatorID, int>> planned_execution;
	// next action of the planned execution from each reference node
	using next_action_t = std::vector<decltype(planned_execution)::const_iterator>;
	const next_action_t next_action;
	static auto initialize_next_action(StateRegistry &state_registry, TaskProxy task_proxy, StateID initial_state_id, const std::vector<std::pair<OperatorID, int>> &planned_execution, const std::vector<std::pair<StateID, int>> &reference_nodes)->next_action_t;

	auto create_open_list_factory(const options::Options &opts) const -> std::unique_ptr<OpenListFactory>;

	virtual void initialize() override;
	virtual SearchStatus step() override;

	std::tuple<SearchNode, bool, std::size_t> fetch_next_node();
	virtual std::size_t return_best_index(std::vector<std::unique_ptr<StateOpenList>> &open_lists) = 0;

	virtual auto prune_state(std::size_t, const GlobalState &) const -> bool { return false; }

	auto check_goal_and_set_plan(const GlobalState &state, const SearchSpace &search_space) -> bool;
	void reward_progress();
	void print_checkpoint_line(int g) const;

	class ExpansionDelay {
		double avg_expansion_delay;
		const SearchStatistics &statistics;
		const int additional_expansions;

		std::deque<int> last_delays;
		long long last_delays_sum;
		const std::size_t moving_average_size;
	public:
		ExpansionDelay(const SearchStatistics &statistics, const std::size_t moving_average_size) :
			avg_expansion_delay(0),
			statistics(statistics),
			additional_expansions(0),
			last_delays_sum(0),
			moving_average_size(moving_average_size) {}

		ExpansionDelay(const SearchStatistics &statistics, const std::size_t moving_average_size, double initial_avg_expansion_delay_value, int additional_expansions) :
			avg_expansion_delay(initial_avg_expansion_delay_value),
			statistics(statistics),
			additional_expansions(additional_expansions),
			last_delays(additional_expansions, std::lround(initial_avg_expansion_delay_value)),
			last_delays_sum(std::lround(initial_avg_expansion_delay_value) * moving_average_size),
			moving_average_size(moving_average_size) {}

		void update_expansion_delay(int delay) {
			assert(delay >= 1);
			avg_expansion_delay += (delay - avg_expansion_delay) / (statistics.get_expanded() + additional_expansions);
			if (moving_average_size != 0u) {
				last_delays.push_back(delay);
				last_delays_sum += delay;
				if (last_delays.size() > moving_average_size) {
					last_delays_sum -= last_delays.front();
					last_delays.pop_front();
				}
			}
		}

		virtual auto get_avg_expansion_delay() const -> double {
			if (statistics.get_expanded() + additional_expansions < 2)
				return 1.;
			return moving_average_size == 0u ? avg_expansion_delay : last_delays_sum / static_cast<double>(last_delays.size());
		}

		virtual auto get_total_avg_expansion_delay() const -> double {
			if (statistics.get_expanded() + additional_expansions < 2)
				return 1.;
			return avg_expansion_delay;
		}
	};

	std::unique_ptr<ExpansionDelay> expansion_delay;
	std::vector<int> expansions;
	std::vector<std::unordered_map<StateID, int>> open_list_insertion_time;

	std::shared_ptr<Evaluator> d_eval;

	std::vector<std::unique_ptr<StateOpenList>> open_lists;
	std::vector<std::unique_ptr<SearchSpace>> search_spaces;
	std::vector<int> min_distance;

	std::size_t plan_reference_node_index;

public:
	MultipleInitialStateAStarSearch(const options::Options &opts,
	                                std::shared_ptr<StateRegistry> state_registry, 
	                                std::shared_ptr<AbstractTask> task,
	                                StateID initial_state_id,
	                                std::vector<std::pair<StateID, int>> &&reference_nodes,
	                                std::vector<std::pair<OperatorID, int>> &&planned_execution,
	                                int moving_average_size,
	                                double initial_avg_expansion_delay_value = 0.,
	                                int last_expansions = 0);
	virtual ~MultipleInitialStateAStarSearch() = default;

	virtual void print_statistics() const override;
	void dump_search_space() const;

	auto get_reference_node_index() const -> std::size_t {
		return plan_reference_node_index;
	}

	auto get_expansions_by_reference_node() const -> const std::vector<int> & {
		return expansions;
	}

	auto get_expansion_delay() const -> double {
		return expansion_delay->get_total_avg_expansion_delay();
	}
};
}

#endif
