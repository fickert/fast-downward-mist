#ifndef ONLINE_ONLINE_SEARCH_H
#define ONLINE_ONLINE_SEARCH_H

#include "../option_parser_util.h"
#include "../search_engine.h"

#include "../options/registries.h"
#include "../options/predefinitions.h"

#include <random>

namespace options {
class Options;
}

namespace online {
class OnlineSearch : public SearchEngine {
    const options::ParseTree eval_config;
    /*
      We need to copy the registry and predefinitions here since they live
      longer than the objects referenced in the constructor.
    */
    options::Registry registry;
    options::Predefinitions predefinitions;

    std::vector<FactPair> goal;
	std::vector<std::pair<std::vector<FactPair>, int>> online_goals;
	decltype(online_goals)::const_iterator next_goal;
	const int moving_average_size;

	std::shared_ptr<SearchEngine> subengine;

	struct ReversibleAction {
		ReversibleAction(int index)
			: forward(true),
			  index(index),
			  previous_state_id(StateID::no_state) {}

		ReversibleAction(int index, StateID previous_state_id)
			: forward(false),
			  index(index),
			  previous_state_id(previous_state_id) {}

		const bool forward;
		const int index;
		const StateID previous_state_id;
	};

	std::vector<ReversibleAction> last_plan;
	// points one position behind the last action to be applied from the last plan
	std::vector<decltype(last_plan)::const_iterator> last_reference_node_positions;
	// points one position behind the last action that was applied from the last plan
	decltype(last_plan)::const_iterator last_plan_executed;
	int last_plan_available_search_time;

	double last_expansion_delay;
	int last_expansions;

	using internal_reference_node_t = std::tuple<StateID, int, decltype(last_plan)::const_iterator>;
	using search_reference_node_t = std::pair<StateID, int>;

#ifndef NDEBUG
	int debug_current_time;
#endif

	struct OnlineSearchStatistics {
		OnlineSearchStatistics() :
			execution_time_expansions(0),
			parallel_time_expansions(0),
			waiting_time_expansions(0),
			expansions_after_first_phase(0),
			correct_reference_node_expansions(0),
			last_deviation_time(0),
			last_deviation_actions(0),
			last_total_time(0),
			last_total_actions(0) {}

		int execution_time_expansions;
		int parallel_time_expansions;
		int waiting_time_expansions;

		int expansions_after_first_phase;
		int correct_reference_node_expansions;

		int last_deviation_time;
		int last_deviation_actions;
		int last_total_time;
		int last_total_actions;
	} rts_statistics;

	auto get_current_effective_expansions() const -> int {
		assert(rts_statistics.execution_time_expansions >= rts_statistics.parallel_time_expansions);
		assert(statistics.get_expanded() >= rts_statistics.parallel_time_expansions);
		return rts_statistics.waiting_time_expansions + rts_statistics.execution_time_expansions - rts_statistics.parallel_time_expansions + statistics.get_expanded();
	}

	Plan overall_plan;
	GlobalState current_state;

	auto get_evaluator() -> std::shared_ptr<Evaluator>;
	auto get_subengine_options() -> Options;

	void start_new_search(std::vector<internal_reference_node_t> &&reference_nodes);
	void start_new_search(StateID initial_state_id);

	std::mt19937 rng;

	enum class ReplanStrategy {
		STOP_EXECUTION,
		FINISH_EXECUTION,
		REFERENCE_NODES,
		REFERENCE_NODES_SIMPLE,
		SIMULATE_ALL
	} const replan_strategy;

	auto using_reference_nodes_replan_strategy() const -> bool {
		return replan_strategy == ReplanStrategy::REFERENCE_NODES || replan_strategy == ReplanStrategy::REFERENCE_NODES_SIMPLE;
	}

	enum class ReferenceNodeSamplingMethod {
		NORMAL_RANDOM,
		UNIFORM_RANDOM,
		UNIFORM,
		SINGLE_BEST_ESTIMATE,
		FIXED_LATENCY
	} const reference_node_sampling_method;

	auto using_single_node_sampling_method() const -> bool {
		return reference_node_sampling_method == ReferenceNodeSamplingMethod::SINGLE_BEST_ESTIMATE ||
		       reference_node_sampling_method == ReferenceNodeSamplingMethod::FIXED_LATENCY;
	}

	const int latency;

	const double single_best_estimate_factor;

	const int num_reference_nodes;

	const bool reset_expansion_delay;

	void apply_filter(const std::set<std::size_t> &selected, std::vector<internal_reference_node_t> &reference_nodes) const;

	template<class ValueType>
	auto find_closest_reference_node(ValueType x, const std::vector<internal_reference_node_t> &reference_nodes) const -> std::size_t {
		assert(!reference_nodes.empty());
		const auto lower_bound = std::lower_bound(std::begin(reference_nodes), std::end(reference_nodes), x, [](const auto &reference_node, const auto &value) {
			return std::get<int>(reference_node) < value;
		});
		return std::distance(std::begin(reference_nodes),
			lower_bound == std::begin(reference_nodes) || (lower_bound != std::end(reference_nodes) && std::abs(std::get<int>(*lower_bound) - x) <= std::abs(std::get<int>(*std::next(lower_bound, -1)) - x)) ?
			lower_bound : std::next(lower_bound, -1));
	}

	template<class Distribution>
	void filter_reference_nodes_random(Distribution &distribution, std::vector<internal_reference_node_t> &reference_nodes) {
		assert(reference_nodes.size() > 1u);
		assert(static_cast<int>(reference_nodes.size()) > num_reference_nodes);

		auto selected = std::set<std::size_t>{reference_nodes.size() - 1};
		// while (selected.size() < num_reference_nodes) {
		for (auto i = 0; i < num_reference_nodes - 1; ++i)
			selected.insert(find_closest_reference_node(distribution(rng), reference_nodes));
		apply_filter(selected, reference_nodes);
	}

	void filter_reference_nodes_normal_distribution(double mean, std::vector<internal_reference_node_t> &reference_nodes);
	void filter_reference_nodes_uniform_distribution(std::vector<internal_reference_node_t> &reference_nodes);
	void filter_reference_nodes_uniform_intervals(std::vector<internal_reference_node_t> &reference_nodes) const;

	virtual SearchStatus step() override;

	auto get_successor_state(const GlobalState &state, const ReversibleAction &reversible_action) const -> GlobalState;
	auto apply_reversible(const ReversibleAction &reversible_action) -> ReversibleAction;
	auto simulate_execution(std::vector<ReversibleAction>::const_iterator plan_begin, std::vector<ReversibleAction>::const_iterator plan_end) -> int;
	auto get_reference_node_index() const -> int;
	auto collect_reference_nodes_and_start_next_search(int initial_reference_state_distance) -> SearchStatus;
	auto simulate_execution_and_get_next_plan() -> std::vector<ReversibleAction>;

public:
    OnlineSearch(const options::Options &opts, options::Registry &registry,
                   const options::Predefinitions &predefinitions);

	virtual void print_statistics() const override;
};
}

#endif
