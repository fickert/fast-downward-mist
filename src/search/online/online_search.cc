#include "online_search.h"

#include "../option_parser.h"
#include "../plugin.h"

#include <iostream>
#include <numeric>

#include "multiple_initial_states_astar_search.h"
#include "pruning_misastar_search.h"
#include "recoverability_misastar_search.h"
#include "../tasks/modified_goals_task.h"
#include "../tasks/root_task.h"
#include "../evaluators/weighted_evaluator.h"
#include "../search_engines/eager_search.h"
#include "../search_engines/search_common.h"
#include "../pruning/null_pruning_method.h"
#include "ff_distance_wrapper.h"

using namespace std;

namespace online {

auto get_goal_facts(const AbstractTask &task) -> std::vector<FactPair> {
	auto goal_facts = std::vector<FactPair>();
	for (auto i = 0; i < task.get_num_goals(); ++i)
		goal_facts.push_back(task.get_goal_fact(i));
	return goal_facts;
}

auto get_online_goals(const AbstractTask &task) -> std::vector<std::pair<std::vector<FactPair>, int>> {
	auto online_goals = std::vector<std::pair<std::vector<FactPair>, int>>();
	for (auto i = 0; i < task.get_num_online_goals(); ++i)
		online_goals.emplace_back(task.get_online_goal(i));
	std::sort(std::begin(online_goals), std::end(online_goals), [](const auto &lhs, const auto &rhs) { return lhs.second < rhs.second; });
	return online_goals;
}

OnlineSearch::OnlineSearch(const Options &opts, options::Registry &registry,
                               const options::Predefinitions &predefinitions)
    : SearchEngine(opts),
      eval_config(opts.get<ParseTree>("eval")),
      registry(registry),
      predefinitions(predefinitions),
      goal(get_goal_facts(*task)),
      online_goals(get_online_goals(*task)),
      next_goal(std::cbegin(online_goals)),
      moving_average_size(opts.get<int>("moving_average_size")),
      last_plan_available_search_time(0),
      last_expansions(0),
#ifndef NDEBUG
      debug_current_time(0),
#endif
      current_state(state_registry->get_initial_state()),
      replan_strategy(ReplanStrategy(opts.get_enum("replan_strategy"))),
      reference_node_sampling_method(ReferenceNodeSamplingMethod(opts.get_enum("reference_node_sampling_method"))),
      latency(opts.get<int>("latency")),
      single_best_estimate_factor(opts.get<double>("single_best_estimate_factor")),
      // note that reference_node_sampling_method must already be initialized here
      num_reference_nodes(using_single_node_sampling_method() ? 1 : opts.get<int>("num_reference_nodes")),
      reset_expansion_delay(opts.get<bool>("reset_expansion_delay")) {
	if (replan_strategy == ReplanStrategy::SIMULATE_ALL && online_goals.size() != 1u) {
		std::cerr << "Simulating search for all deviation points only works with a single online goal." << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
	}
	if (opts.get<int>("seed") != -1)
		rng.seed(opts.get<int>("seed"));
	assert(last_plan.empty());
	last_plan_executed = std::cend(last_plan);
	if (using_reference_nodes_replan_strategy()) {
		auto initial_reference_nodes = std::vector<internal_reference_node_t>{internal_reference_node_t(current_state.get_id(), 0, std::cend(last_plan))};
		start_new_search(std::move(initial_reference_nodes));
	} else {
		start_new_search(current_state.get_id());
	}
	std::cout << "initialized online re-planning search, the goal schedule is:" << std::endl;
	std::cout << "  initial: " << task->get_num_goals() << std::endl;
	for (auto i = 0; i < task->get_num_online_goals(); ++i) {
		const auto online_goal = task->get_online_goal(i);
		std::cout << "  after " << online_goal.second << " expansions: " << online_goal.first.size() << std::endl;
	}
}

auto OnlineSearch::get_evaluator() -> std::shared_ptr<Evaluator> {
	auto current_config = eval_config;
	const auto modified_goals_it = current_config.append_child(current_config.begin(), options::ParseNode("modified_goals", "transform"));
	const auto variables_it = current_config.append_child(modified_goals_it, options::ParseNode("list", "variables"));
	const auto values_it = current_config.append_child(modified_goals_it, options::ParseNode("list", "values"));
	for (const auto goal_fact : goal) {
		current_config.append_child(variables_it, options::ParseNode(std::to_string(goal_fact.var), ""));
		current_config.append_child(values_it, options::ParseNode(std::to_string(goal_fact.value), ""));
	}
	OptionParser parser(current_config, registry, predefinitions, false);
	return parser.start_parsing<std::shared_ptr<Evaluator>>();
}

auto OnlineSearch::get_subengine_options() -> Options {
	auto subengine_options = options::Options();
	const auto evaluator = get_evaluator();
	subengine_options.set("eval", evaluator);
	if (is_unit_cost) {
		subengine_options.set("d_eval", evaluator);
	} else {
		if (eval_config.begin()->value == std::string("ff")) {
			subengine_options.set<std::shared_ptr<Evaluator>>("d_eval", std::make_shared<FFDistanceWrapper>(std::static_pointer_cast<ff_heuristic::FFHeuristic>(evaluator)));
		} else {
			std::cerr << "distance evaluator not implemented for anything but the ff heuristic" << std::endl;
			// TODO: create the distance evaluator from the same configuration as the evaluator itself, but with a unit-cost transformation
			utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
		}
	}
	auto goal_copy = goal;
	subengine_options.set("transform", std::make_shared<extra_tasks::ModifiedGoalsTask>(task, std::move(goal_copy)));
	subengine_options.set<int>("cost_type", cost_type);
	subengine_options.set("max_time", max_time); // TODO: subtract elapsed time
	subengine_options.set("bound", bound);
	subengine_options.set("reopen_closed", true);
	return subengine_options;
}

void OnlineSearch::start_new_search(std::vector<internal_reference_node_t> &&reference_nodes) {
	auto subengine_options = get_subengine_options();
	auto modified_task = subengine_options.get<std::shared_ptr<extra_tasks::ModifiedGoalsTask>>("transform");

	auto all_reference_nodes = reference_nodes;

#ifndef NDEBUG
	std::cout << "################ REFERENCE NODE SCHEDULE: ################" << std::endl;
	for (const auto &reference_node : reference_nodes)
		std::cout << "  " << std::get<int>(reference_node) << std::endl;
	std::cout << "##########################################################" << std::endl;
#endif

	if (num_reference_nodes != 0 && static_cast<int>(reference_nodes.size()) > num_reference_nodes) {
		switch (reference_node_sampling_method) {
		case ReferenceNodeSamplingMethod::NORMAL_RANDOM: {
			auto eval_context = EvaluationContext(current_state);
			if (eval_config.begin()->value == std::string("ff")) {
				// we need to evaluate eval before d_eval when using FF
				auto eval = subengine_options.get<std::shared_ptr<Evaluator>>("eval");
				eval->compute_result(eval_context);
			}
			auto d_eval = subengine_options.get<std::shared_ptr<Evaluator>>("d_eval");
			auto result = d_eval->compute_result(eval_context);
			assert(!result.is_infinite());
			const auto estimated_planning_time = result.get_evaluator_value() * last_expansion_delay > std::numeric_limits<int>::max() * 1. ?
				std::numeric_limits<int>::max() : static_cast<int>(std::lround(result.get_evaluator_value() * last_expansion_delay));
			filter_reference_nodes_normal_distribution(estimated_planning_time, reference_nodes);
			break;
		}
		case ReferenceNodeSamplingMethod::UNIFORM_RANDOM:
			filter_reference_nodes_uniform_distribution(reference_nodes);
			break;
		case ReferenceNodeSamplingMethod::UNIFORM:
			filter_reference_nodes_uniform_intervals(reference_nodes);
			break;
		case ReferenceNodeSamplingMethod::SINGLE_BEST_ESTIMATE: {
			auto eval_context = EvaluationContext(current_state);
			if (eval_config.begin()->value == std::string("ff")) {
				// we need to evaluate eval before d_eval when using FF
				auto eval = subengine_options.get<std::shared_ptr<Evaluator>>("eval");
				eval->compute_result(eval_context);
			}
			auto d_eval = subengine_options.get<std::shared_ptr<Evaluator>>("d_eval");
			auto result = d_eval->compute_result(eval_context);
			assert(!result.is_infinite());
			const auto estimated_planning_time = result.get_evaluator_value() * last_expansion_delay > std::numeric_limits<int>::max() * 1. ?
				std::numeric_limits<int>::max() : static_cast<int>(std::lround(result.get_evaluator_value() * last_expansion_delay * single_best_estimate_factor));
			const auto closest_reference_node_index = find_closest_reference_node(estimated_planning_time, reference_nodes);
			auto closest_reference_node = std::move(reference_nodes[closest_reference_node_index]);
			reference_nodes.clear();
			reference_nodes.emplace_back(std::move(closest_reference_node));
			break;
		}
		case ReferenceNodeSamplingMethod::FIXED_LATENCY: {
			const auto closest_reference_node_index = find_closest_reference_node(latency, reference_nodes);
			auto closest_reference_node = std::move(reference_nodes[closest_reference_node_index]);
			reference_nodes.clear();
			reference_nodes.emplace_back(std::move(closest_reference_node));
			break;
		}
		default:
			std::cerr << "Unexpected reference node sampling method: " << static_cast<int>(reference_node_sampling_method) << std::endl;
			utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
		}
	}
	auto search_reference_nodes = std::vector<search_reference_node_t>();
	assert(last_reference_node_positions.empty());
	for (const auto &reference_node : reference_nodes) {
		search_reference_nodes.emplace_back(std::get<StateID>(reference_node), std::get<int>(reference_node));
		last_reference_node_positions.emplace_back(std::get<decltype(last_plan)::const_iterator>(reference_node));
	}

	auto planned_execution = std::vector<std::pair<OperatorID, int>>();
	assert(!all_reference_nodes.empty());
	planned_execution.reserve(all_reference_nodes.size() - 1);
	auto reference_node_it = std::cbegin(all_reference_nodes);
	for (auto plan_it = last_plan_executed; plan_it != std::cend(last_plan); ++plan_it) {
		assert(plan_it->forward);
		planned_execution.emplace_back(OperatorID(plan_it->index), std::get<int>(*reference_node_it));
		assert(reference_node_it != std::cend(all_reference_nodes));
		++reference_node_it;
	}

	if (replan_strategy == ReplanStrategy::REFERENCE_NODES_SIMPLE) {
		subengine = reset_expansion_delay || last_expansions == 0 ?
			std::make_shared<PruningMISAStarSearch>(subengine_options, state_registry, modified_task, current_state.get_id(), std::move(search_reference_nodes), std::move(planned_execution), moving_average_size) :
			std::make_shared<PruningMISAStarSearch>(subengine_options, state_registry, modified_task, current_state.get_id(), std::move(search_reference_nodes), std::move(planned_execution), moving_average_size, last_expansion_delay, last_expansions);
	} else {
		subengine = reset_expansion_delay || last_expansions == 0 ?
			std::make_shared<RecoverabilityMISAStarSearch>(subengine_options, state_registry, modified_task, current_state.get_id(), std::move(search_reference_nodes), std::move(planned_execution), moving_average_size) :
			std::make_shared<RecoverabilityMISAStarSearch>(subengine_options, state_registry, modified_task, current_state.get_id(), std::move(search_reference_nodes), std::move(planned_execution), moving_average_size, last_expansion_delay, last_expansions);
	}
}

void OnlineSearch::start_new_search(StateID initial_state_id) {
	assert(last_reference_node_positions.empty());
	assert(replan_strategy == ReplanStrategy::STOP_EXECUTION
		|| replan_strategy == ReplanStrategy::FINISH_EXECUTION
		|| replan_strategy == ReplanStrategy::SIMULATE_ALL);
	last_reference_node_positions.emplace_back(replan_strategy == ReplanStrategy::STOP_EXECUTION ? last_plan_executed : std::cend(last_plan));

	auto subengine_options = get_subengine_options();
	const auto temp = search_common::create_astar_open_list_factory_and_f_eval(subengine_options);
	subengine_options.set("open", temp.first);
	subengine_options.set("f_eval", temp.second);
	subengine_options.set<std::vector<std::shared_ptr<Evaluator>>>("preferred", {});
	subengine_options.set<std::shared_ptr<PruningMethod>>("pruning", std::make_shared<null_pruning_method::NullPruningMethod>());
	subengine = std::make_shared<eager_search::EagerSearch>(subengine_options, state_registry, subengine_options.get<std::shared_ptr<extra_tasks::ModifiedGoalsTask>>("transform"), initial_state_id, true);
}

void OnlineSearch::apply_filter(const std::set<std::size_t> &selected, std::vector<internal_reference_node_t> &reference_nodes) const {
	auto resulting_reference_nodes = std::vector<internal_reference_node_t>();
	resulting_reference_nodes.reserve(selected.size());
	for (auto selected_index : selected)
		resulting_reference_nodes.emplace_back(std::move(reference_nodes[selected_index]));
	reference_nodes = std::move(resulting_reference_nodes);
}

void OnlineSearch::filter_reference_nodes_normal_distribution(double mean, std::vector<internal_reference_node_t> &reference_nodes) {
	mean = std::max(std::get<int>(reference_nodes.front()) * 1., mean);
	mean = std::min(std::get<int>(reference_nodes.back()) * 1., mean);
	auto deviation = std::min(mean - std::get<int>(reference_nodes.front()), std::get<int>(reference_nodes.back()) - mean) / 3.;
	assert(deviation >= 0);
	deviation = std::max(deviation, 0.001);
	assert(deviation > 0);
	auto distribution = std::normal_distribution<double>(mean, deviation);
	filter_reference_nodes_random(distribution, reference_nodes);
}

void OnlineSearch::filter_reference_nodes_uniform_distribution(std::vector<internal_reference_node_t> &reference_nodes) {
	assert(reference_nodes.size() > 1u);
	assert(static_cast<int>(reference_nodes.size()) > num_reference_nodes);
	auto distribution = std::uniform_int_distribution(0, std::get<int>(reference_nodes.back()));
	filter_reference_nodes_random(distribution, reference_nodes);
}

void OnlineSearch::filter_reference_nodes_uniform_intervals(std::vector<internal_reference_node_t> &reference_nodes) const {
	auto selected = std::set<std::size_t>{reference_nodes.size() - 1};
	for (auto i = 0; i < num_reference_nodes - 1; ++i)
		selected.insert(find_closest_reference_node(i * static_cast<double>(std::get<int>(reference_nodes.back())) / num_reference_nodes, reference_nodes));
	apply_filter(selected, reference_nodes);
}

auto OnlineSearch::collect_reference_nodes_and_start_next_search(int initial_reference_state_distance) -> SearchStatus {
	switch (replan_strategy) {
	case ReplanStrategy::STOP_EXECUTION:
		start_new_search(current_state.get_id());
		return IN_PROGRESS;
	case ReplanStrategy::FINISH_EXECUTION: {
		auto state = current_state;
		for (auto plan_it = last_plan_executed; plan_it != std::cend(last_plan); ++plan_it) {
			assert(plan_it->forward);
			state = state_registry->get_successor_state(state, task_proxy.get_operators()[plan_it->index]);
		}
		start_new_search(state.get_id());
		return IN_PROGRESS;
	}
	case ReplanStrategy::REFERENCE_NODES_SIMPLE: // fall through to REFERENCE_NODES case
	case ReplanStrategy::REFERENCE_NODES: {
		auto current_reference_state_distance = initial_reference_state_distance;
		auto current_reference_state = current_state;
		auto reference_nodes = std::vector<internal_reference_node_t>();
		auto plan_it = last_plan_executed;
		while (true) {
			reference_nodes.emplace_back(current_reference_state.get_id(), current_reference_state_distance, plan_it);
			if (plan_it == std::cend(last_plan))
				break;
			current_reference_state = get_successor_state(current_reference_state, *plan_it);
			current_reference_state_distance += task->get_operator_cost(plan_it->index, false) * task->get_execution_time();
			++plan_it;
		}
		start_new_search(std::move(reference_nodes));
		return IN_PROGRESS;
	}
	case ReplanStrategy::SIMULATE_ALL: {
		auto state = current_state;
		auto plan_it = last_plan_executed;
		auto execution_time = std::accumulate(std::cbegin(last_plan), plan_it, 0, [this](const auto sum, const auto& action) {
			return sum + task->get_operator_cost(action.index, false) * task->get_execution_time();
		});
		const auto total_execution_time = std::accumulate(std::cbegin(last_plan), std::cend(last_plan), 0, [this](const auto sum, const auto& action) {
			return sum + task->get_operator_cost(action.index, false) * task->get_execution_time();
		});
		auto execution_actions = static_cast<int>(std::distance(std::cbegin(last_plan), plan_it));
		const auto total_execution_actions = static_cast<int>(last_plan.size());
		auto added_execution_time = 0;
		auto best_deviation_point = plan_it;
		auto best_total_time = std::numeric_limits<int>::max();
		auto best_suffix_plan = Plan();
		auto best_statistics = SearchStatistics();
#ifndef NDEBUG
		auto best_search_start_time = 0;
		auto best_search_end_time = 0;
#endif
		while (true) {
			// suppress output from the search
			std::cout.setstate(std::ios_base::failbit);
			start_new_search(state.get_id());
			// start_new_search will set last_reference_node_positions to a non-sensible value ...
			// ... but we're not using it here anyway, so we can already clear it for the next iteration
			last_reference_node_positions.clear();
			assert(subengine && subengine->get_status() == IN_PROGRESS);
			subengine->search();
			assert(subengine->get_status() != IN_PROGRESS);
			std::cout.clear();
			if (subengine->get_status() != SOLVED) {
				std::cerr << "Subengine unexpectedly terminated with failure." << std::endl;
				utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
			}

			const auto search_start_time = get_current_effective_expansions() - last_plan_available_search_time;
			const auto search_end_time = search_start_time + std::max(last_plan_available_search_time + added_execution_time, subengine->get_statistics().get_expanded());
			const auto &next_plan = subengine->get_plan();
			const auto total_time = search_end_time + std::accumulate(std::cbegin(next_plan), std::cend(next_plan), 0, [this](const auto sum, const auto &action) {
				return sum + task->get_operator_cost(action.get_index(), false) * task->get_execution_time();
			});
#ifndef NDEBUG
			std::cout << "Finished planning after " << subengine->get_statistics().get_expanded() << " expansions, plan length is " << next_plan.size() << std::endl;
#endif
			std::cout << "Effective total expansions after executing a fraction of " << execution_time / static_cast<double>(total_execution_time)
			          << " of the last prefix (" << execution_actions << " of " << total_execution_actions << " actions, or a fraction of "
			          << execution_actions / static_cast<double>(total_execution_actions) << "): " << total_time << " ("
			          << total_time - online_goals.front().second << " after online goals appeared)" << std::endl;

			if (total_time < best_total_time) {
				best_deviation_point = plan_it;
				best_total_time = total_time;
				best_suffix_plan = next_plan;
				best_statistics = subengine->get_statistics();
#ifndef NDEBUG
				best_search_start_time = search_start_time;
				best_search_end_time = search_end_time;
#endif
			}

			if (plan_it == std::cend(last_plan))
				break;
			assert(plan_it->forward);
			state = state_registry->get_successor_state(state, task_proxy.get_operators()[plan_it->index]);
			execution_time += task->get_operator_cost(plan_it->index, false) * task->get_execution_time();
			added_execution_time += task->get_operator_cost(plan_it->index, false) * task->get_execution_time();
			++execution_actions;
			++plan_it;
		}

		// pretend we were able to choose the perfect deviation point beforehand:
		// - simulate execution of the old plan to the deviation point
		// - simulate execution of the new plan

		statistics.inc_expanded(best_statistics.get_expanded());
		statistics.inc_evaluated_states(best_statistics.get_evaluated_states());
		statistics.inc_evaluations(best_statistics.get_evaluations());
		statistics.inc_generated(best_statistics.get_generated());
		statistics.inc_generated_ops(best_statistics.get_generated_ops());
		statistics.inc_reopened(best_statistics.get_reopened());

		// simulate execution until reference node and update statistics
		const auto last_plan_execution_time = simulate_execution(last_plan_executed, best_deviation_point);

		rts_statistics.last_deviation_time = std::accumulate(std::cbegin(last_plan), best_deviation_point, 0, [this](const auto sum, const auto& action) {
			return sum + task->get_operator_cost(action.index, false) * task->get_execution_time();
			});
		rts_statistics.last_deviation_actions = std::distance(std::cbegin(last_plan), best_deviation_point);
		rts_statistics.last_total_time = std::accumulate(std::cbegin(last_plan), std::cend(last_plan), 0, [this](const auto sum, const auto& action) {
			return sum + task->get_operator_cost(action.index, false) * task->get_execution_time();
			});
		rts_statistics.last_total_actions = std::distance(std::cbegin(last_plan), std::cend(last_plan));

#ifndef NDEBUG
		auto debug_state = current_state;
		for (auto op_id : best_suffix_plan) {
			for (auto precondition : task_proxy.get_operators()[op_id.get_index()].get_preconditions())
				assert(debug_state[precondition.get_variable().get_id()] == precondition.get_value());
			debug_state = state_registry->get_successor_state(debug_state, task_proxy.get_operators()[op_id.get_index()]);
		}
#endif

		auto transformed_plan = std::vector<ReversibleAction>();
		transformed_plan.reserve(best_suffix_plan.size());
		std::transform(std::cbegin(best_suffix_plan), std::cend(best_suffix_plan), std::back_inserter(transformed_plan), [](const auto& op_id) {
			return ReversibleAction(op_id.get_index());
		});

		rts_statistics.parallel_time_expansions += std::min(last_plan_available_search_time + last_plan_execution_time, best_statistics.get_expanded());
		rts_statistics.execution_time_expansions += last_plan_execution_time;

#ifndef NDEBUG
		debug_current_time = std::max(best_search_start_time + best_statistics.get_expanded(), best_search_end_time);
		assert(debug_current_time == get_current_effective_expansions());
#endif

		assert(next_goal == std::cend(online_goals));
		rts_statistics.execution_time_expansions += simulate_execution(std::cbegin(transformed_plan), std::cend(transformed_plan));
		set_plan(overall_plan);
#ifndef NDEBUG
		// verify plan
		debug_state = state_registry->get_initial_state();
		for (auto op_id : overall_plan) {
			for (auto precondition : task_proxy.get_operators()[op_id.get_index()].get_preconditions())
				assert(debug_state[precondition.get_variable().get_id()] == precondition.get_value());
			debug_state = state_registry->get_successor_state(debug_state, task_proxy.get_operators()[op_id.get_index()]);
		}
		for (const auto goal_fact : goal)
			assert(debug_state[goal_fact.var] == goal_fact.value);
#endif
		std::cout << "Finished simulating all " << std::distance(last_plan_executed, std::cend(last_plan)) + 1 << " possible deviation points." << std::endl;
		return SOLVED;
	}
	default:
		std::cerr << "Unknown replan strategy: " << static_cast<int>(replan_strategy) << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
	}
}

SearchStatus OnlineSearch::step() {
	assert(subengine && subengine->get_status() == IN_PROGRESS);
	subengine->search();
	assert(subengine->get_status() != IN_PROGRESS);
	if (subengine->get_status() != SOLVED)
		return subengine->get_status(); // TIMEOUT or FAILED

	auto next_plan = simulate_execution_and_get_next_plan();

	auto updated_goal = false;
	auto next_search_start = get_current_effective_expansions();

	// collect all online goals until current effective time, unless it's still empty, then also collect the first one in the future
	for (; next_goal != std::cend(online_goals) && (next_goal->second <= get_current_effective_expansions() || !updated_goal); ++next_goal) {
		goal.insert(std::end(goal), std::begin(next_goal->first), std::end(next_goal->first));
		next_search_start = std::max(next_search_start, next_goal->second);
		updated_goal = true;
	}

	if (!updated_goal) {
		assert(next_goal == std::cend(online_goals));
		rts_statistics.execution_time_expansions += simulate_execution(std::begin(next_plan), std::end(next_plan));
		set_plan(overall_plan);
#ifndef NDEBUG
		// verify plan
		auto debug_state = state_registry->get_initial_state();
		for (auto op_id : overall_plan) {
			for (auto precondition : task_proxy.get_operators()[op_id.get_index()].get_preconditions())
				assert(debug_state[precondition.get_variable().get_id()] == precondition.get_value());
			debug_state = state_registry->get_successor_state(debug_state, task_proxy.get_operators()[op_id.get_index()]);
		}
		for (const auto goal_fact : goal)
			assert(debug_state[goal_fact.var] == goal_fact.value);
#endif
		return SOLVED;
	}

#ifndef NDEBUG
	std::cout << "next search starts at " << next_search_start << " total expansions." << std::endl;
#endif

	const auto available_execution_time = next_search_start - get_current_effective_expansions();
	auto num_actions_to_apply = 0;
	auto used_execution_time = 0;
	while (num_actions_to_apply < static_cast<int>(next_plan.size()) && used_execution_time < available_execution_time) {
		used_execution_time += task->get_operator_cost(next_plan[num_actions_to_apply].index, false) * task->get_execution_time();
		++num_actions_to_apply;
	}

#ifndef NDEBUG
	std::cout << "simulated the execution of " << num_actions_to_apply << " actions (" << used_execution_time << " expansions) of the next plan." << std::endl;
#endif

	assert(num_actions_to_apply == static_cast<int>(next_plan.size()) || get_current_effective_expansions() + used_execution_time >= next_search_start);

	// update time
	const auto next_plan_end = std::next(std::cbegin(next_plan), num_actions_to_apply);
	const auto actual_execution_time = simulate_execution(std::cbegin(next_plan), next_plan_end);
	assert(actual_execution_time == used_execution_time);
	(void) actual_execution_time;
	last_plan_available_search_time = std::max(get_current_effective_expansions() + actual_execution_time - next_search_start, 0);
	rts_statistics.execution_time_expansions += actual_execution_time;
#ifndef NDEBUG
	debug_current_time += actual_execution_time;
	assert(debug_current_time == get_current_effective_expansions());
#endif

	if (next_plan_end == std::cend(next_plan)) {
		rts_statistics.waiting_time_expansions += std::max(next_search_start - get_current_effective_expansions(), 0);
#ifndef NDEBUG
		if (next_search_start > debug_current_time)
			debug_current_time += next_search_start - debug_current_time;
		assert(debug_current_time == get_current_effective_expansions());
#endif
	}

	assert(next_search_start <= get_current_effective_expansions());
	last_plan = std::move(next_plan);
	last_plan_executed = std::next(std::cbegin(last_plan), num_actions_to_apply);
	last_reference_node_positions.clear();
	return collect_reference_nodes_and_start_next_search(get_current_effective_expansions() - next_search_start);
}

auto OnlineSearch::get_successor_state(const GlobalState &state, const ReversibleAction &reversible_action) const -> GlobalState {
	return reversible_action.forward ?
		state_registry->get_successor_state(state, task_proxy.get_operators()[reversible_action.index]) :
		state_registry->lookup_state(reversible_action.previous_state_id);
}

auto OnlineSearch::apply_reversible(const ReversibleAction &reversible_action) -> ReversibleAction {
	if (reversible_action.forward) {
#ifndef NDEBUG
		for (auto precondition : task_proxy.get_operators()[reversible_action.index].get_preconditions())
			assert(current_state[precondition.get_variable().get_id()] == precondition.get_value());
#endif
		const auto previous_state_id = current_state.get_id();
		current_state = state_registry->get_successor_state(current_state, task_proxy.get_operators()[reversible_action.index]);
		overall_plan.emplace_back(OperatorID(reversible_action.index));
		return ReversibleAction(reversible_action.index, previous_state_id);
	}
	assert(overall_plan.back().get_index() == reversible_action.index);
	overall_plan.pop_back();
#ifndef NDEBUG
	const auto successor_state_id = current_state.get_id();
#endif
	current_state = state_registry->lookup_state(reversible_action.previous_state_id);
#ifndef NDEBUG
	for (auto precondition : task_proxy.get_operators()[reversible_action.index].get_preconditions())
		assert(current_state[precondition.get_variable().get_id()] == precondition.get_value());
	const auto successor_state = state_registry->get_successor_state(current_state, task_proxy.get_operators()[reversible_action.index]);
	assert(successor_state.get_id() == successor_state_id);
#endif
	return ReversibleAction(reversible_action.index);
}

auto OnlineSearch::simulate_execution(std::vector<ReversibleAction>::const_iterator plan_begin, std::vector<ReversibleAction>::const_iterator plan_end) -> int {
	auto execution_time = 0;
	for (auto plan_it = plan_begin; plan_it != plan_end; ++plan_it) {
		apply_reversible(*plan_it);
		execution_time += task->get_operator_cost(plan_it->index, false) * task->get_execution_time();
	}
	return execution_time;
}

int OnlineSearch::get_reference_node_index() const {
	if (using_reference_nodes_replan_strategy())
		return std::static_pointer_cast<MultipleInitialStateAStarSearch>(subengine)->get_reference_node_index();
	assert(last_reference_node_positions.size() == 1u);
	return 0;
}

auto OnlineSearch::simulate_execution_and_get_next_plan() -> std::vector<ReversibleAction> {
	const auto last_search_start_time = get_current_effective_expansions() - last_plan_available_search_time;
	// update statistics
	const SearchStatistics &current_stats = subengine->get_statistics();
	statistics.inc_expanded(current_stats.get_expanded());
	statistics.inc_evaluated_states(current_stats.get_evaluated_states());
	statistics.inc_evaluations(current_stats.get_evaluations());
	statistics.inc_generated(current_stats.get_generated());
	statistics.inc_generated_ops(current_stats.get_generated_ops());
	statistics.inc_reopened(current_stats.get_reopened());

	const auto next_plan = subengine->get_plan();
#ifndef NDEBUG
	std::cout << "finished planning after " << current_stats.get_expanded() << " expansions (" << last_search_start_time + current_stats.get_expanded() << " total expansions)." << std::endl;
	std::cout << "next plan has length " << next_plan.size() << " and starts at reference node #" << get_reference_node_index() << std::endl;
#endif
	const auto current_reference_node_index = get_reference_node_index();
	if (using_reference_nodes_replan_strategy() && last_search_start_time > 0) {
		rts_statistics.expansions_after_first_phase += current_stats.get_expanded();
		rts_statistics.correct_reference_node_expansions += std::static_pointer_cast<MultipleInitialStateAStarSearch>(subengine)->get_expansions_by_reference_node()[current_reference_node_index];
	}

	assert(!last_reference_node_positions.empty());
	assert(static_cast<int>(last_reference_node_positions.size()) > current_reference_node_index);

	// simulate execution until reference node and update statistics
	const auto last_plan_execution_time = simulate_execution(last_plan_executed, last_reference_node_positions.at(current_reference_node_index));
	const auto last_plan_end_time = last_search_start_time + last_plan_available_search_time + last_plan_execution_time;

	rts_statistics.last_deviation_time = std::accumulate(std::cbegin(last_plan), last_reference_node_positions.at(current_reference_node_index), 0, [this](const auto sum, const auto &action) {
		return sum + task->get_operator_cost(action.index, false) * task->get_execution_time();
	});
	rts_statistics.last_deviation_actions = std::distance(std::cbegin(last_plan), last_reference_node_positions.at(current_reference_node_index));
	rts_statistics.last_total_time = std::accumulate(std::cbegin(last_plan), std::cend(last_plan), 0, [this](const auto sum, const auto &action) {
		return sum + task->get_operator_cost(action.index, false) * task->get_execution_time();
	});
	rts_statistics.last_total_actions = std::distance(std::cbegin(last_plan), std::cend(last_plan));

#ifndef NDEBUG
	auto debug_state = current_state;
	for (auto op_id : next_plan) {
		for (auto precondition : task_proxy.get_operators()[op_id.get_index()].get_preconditions())
			assert(debug_state[precondition.get_variable().get_id()] == precondition.get_value());
		debug_state = state_registry->get_successor_state(debug_state, task_proxy.get_operators()[op_id.get_index()]);
	}
#endif

	if (using_reference_nodes_replan_strategy()) {
		last_expansion_delay = std::static_pointer_cast<MultipleInitialStateAStarSearch>(subengine)->get_expansion_delay();
		last_expansions = current_stats.get_expanded();
	}

	if (!using_reference_nodes_replan_strategy() || using_single_node_sampling_method()) {
		// nothing to simulate as execution is halted during planning for STOP_EXECUTION, and for FINISH_EXECUTION the entire plan is executed already
		// for single node sampling, we also halt the execution at the corresponding reference node
		assert(last_reference_node_positions.size() == 1u);
		assert(replan_strategy != ReplanStrategy::FINISH_EXECUTION || last_reference_node_positions.at(current_reference_node_index) == std::cend(last_plan));
		auto transformed_plan = std::vector<ReversibleAction>();
		transformed_plan.reserve(next_plan.size());
		std::transform(std::cbegin(next_plan), std::cend(next_plan), std::back_inserter(transformed_plan), [](const auto &op_id) {
			return ReversibleAction(op_id.get_index());
		});
		rts_statistics.parallel_time_expansions += std::min(last_plan_available_search_time + last_plan_execution_time, current_stats.get_expanded());
		rts_statistics.execution_time_expansions += last_plan_execution_time;
#ifndef NDEBUG
		debug_current_time = std::max(last_search_start_time + current_stats.get_expanded(), last_plan_end_time);
		assert(debug_current_time == get_current_effective_expansions());
#endif
		return transformed_plan;
	}

	// simulate the execution until the current point in time
	// if the execution already went past the reference node, assume we can undo the corresponding actions
	// we can skip undoing actions if the new plan starts with the same actions
	auto next_plan_it = std::cbegin(next_plan);
	auto corrected_plan = std::vector<ReversibleAction>();

	bool plans_match = true;
	auto last_plan_it = last_reference_node_positions.at(current_reference_node_index);
	auto additional_execution_time = 0;
	auto correction_actions = std::vector<ReversibleAction>();
	while (last_plan_end_time + additional_execution_time < last_search_start_time + current_stats.get_expanded() && last_plan_it != std::cend(last_plan)) {
		additional_execution_time += task->get_operator_cost(last_plan_it->index, false) * task->get_execution_time();
		auto reversible_action = apply_reversible(*last_plan_it);
		if (plans_match && next_plan_it != std::cend(next_plan) && last_plan_it->forward && last_plan_it->index == next_plan_it->get_index()) {
			++next_plan_it;
		} else {
			plans_match = false;
			correction_actions.emplace_back(std::move(reversible_action));
		}
		++last_plan_it;
	}

	// with REFERENCE_NODES_SIMPLE, we're not assuming recoverability so there should never be any correction actions
	assert(replan_strategy != ReplanStrategy::REFERENCE_NODES_SIMPLE || correction_actions.empty());

	rts_statistics.parallel_time_expansions += std::min(last_plan_available_search_time + last_plan_execution_time + additional_execution_time, current_stats.get_expanded());
	rts_statistics.execution_time_expansions += last_plan_execution_time + additional_execution_time;

#ifndef NDEBUG
	// the end of the execution should be after the point in time when search finished, or all actions should have been applied
	assert(last_plan_end_time + additional_execution_time >= last_search_start_time + current_stats.get_expanded() || last_plan_it == std::cend(last_plan));
	debug_current_time = std::max(last_search_start_time + current_stats.get_expanded(), last_plan_end_time + additional_execution_time);
	assert(debug_current_time == get_current_effective_expansions());
#endif
	corrected_plan.reserve(correction_actions.size() + std::distance(next_plan_it, std::cend(next_plan)));
	std::move(std::rbegin(correction_actions), std::rend(correction_actions), std::back_inserter(corrected_plan));
	std::transform(next_plan_it, std::cend(next_plan), std::back_inserter(corrected_plan), [](const auto &op_id) {
		return ReversibleAction(op_id.get_index());
	});
	return corrected_plan;
}

void OnlineSearch::print_statistics() const {
    cout << "Cumulative statistics:" << endl;
    statistics.print_detailed_statistics();
	std::cout << "Effective total expansions: " << get_current_effective_expansions() << std::endl;
	if (!online_goals.empty()) {
		assert(get_current_effective_expansions() >= online_goals.front().second);
		std::cout << "Effective total expansions after first online goals appeared: " << get_current_effective_expansions() - online_goals.front().second << std::endl;
		std::cout << "Executed a fraction of " << rts_statistics.last_deviation_time / static_cast<double>(rts_statistics.last_total_time) << " of the last prefix ("
		          << rts_statistics.last_deviation_actions << " of " << rts_statistics.last_total_actions << " actions, or a fraction of "
		          << rts_statistics.last_deviation_actions / static_cast<double>(rts_statistics.last_total_actions) << ")" << std::endl;
	}
	std::cout << "Execution time expansions: " << rts_statistics.execution_time_expansions << std::endl;
	std::cout << "Parallel planning and execution time expansions: " << rts_statistics.parallel_time_expansions << std::endl;
	std::cout << "Waiting time expansions: " << rts_statistics.waiting_time_expansions << std::endl;
	if (rts_statistics.expansions_after_first_phase > 0)
		std::cout << "Correct reference node progress: " << rts_statistics.correct_reference_node_expansions / static_cast<double>(rts_statistics.expansions_after_first_phase) << std::endl;
}

static shared_ptr<SearchEngine> _create_search_engine(OptionParser &parser, Options opts) {
    if (parser.help_mode()) {
        return nullptr;
    } else if (parser.dry_run()) {
        //check if the supplied search engine can be parsed
		OptionParser test_parser(opts.get<ParseTree>("eval"), parser.get_registry(), parser.get_predefinitions(), true);
		test_parser.start_parsing<shared_ptr<Evaluator>>();
        return nullptr;
    } else {
        return make_shared<OnlineSearch>(opts, parser.get_registry(),
                                           parser.get_predefinitions());
    }
}

static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
    parser.document_synopsis("Online re-planning search (replan from the most suitable reference node)", "");
	parser.add_option<ParseTree>("eval", "evaluator for h-value used in subsearch");
	parser.add_option<int>("moving_average_size", "size of the moving average to be considered by the expansion delay (0 to average over all states)", "100", Bounds("0", "infinity"));
	parser.add_enum_option("reference_node_sampling_method", {"NORMAL_RANDOM", "UNIFORM_RANDOM", "UNIFORM", "SINGLE_BEST_ESTIMATE", "FIXED_LATENCY"},
		"sampling method for reference nodes, if SINGLE_BEST_ESTIMATE is used, only the single reference node closest to the estimated end of planning time is used", "UNIFORM");
	parser.add_option<double>("single_best_estimate_factor", "if SINGLE_BEST_ESTIMATE is used, multiply the planning time estimate by this factor", "1.0", Bounds("0", "infinity"));
	parser.add_option<int>("num_reference_nodes", "Number of reference nodes to consider (use 0 for all reference nodes)", "0", Bounds("0", "infinity"));
	parser.add_option<int>("seed", "Set to -1 (default) to use an arbitrary seed, otherwise initialize the random generator with the given seed.", "-1");
	parser.add_option<bool>("reset_expansion_delay", "Reset expansion delay between search iterations.", "false");
    SearchEngine::add_options_to_parser(parser);
	auto opts = parser.parse();
	opts.set("replan_strategy", 2);
	opts.set("latency", -1);
	return _create_search_engine(parser, std::move(opts));
}

static shared_ptr<SearchEngine> _parse_pruning(OptionParser &parser) {
    parser.document_synopsis("Online re-planning search (replan from the most suitable reference node without assuming recoverability)", "");
	parser.add_option<ParseTree>("eval", "evaluator for h-value used in subsearch");
	parser.add_option<int>("moving_average_size", "size of the moving average to be considered by the expansion delay (0 to average over all states)", "100", Bounds("0", "infinity"));
	parser.add_enum_option("reference_node_sampling_method", {"NORMAL_RANDOM", "UNIFORM_RANDOM", "UNIFORM", "SINGLE_BEST_ESTIMATE", "FIXED_LATENCY"},
		"sampling method for reference nodes, if SINGLE_BEST_ESTIMATE is used, only the single reference node closest to the estimated end of planning time is used", "UNIFORM");
	parser.add_option<double>("single_best_estimate_factor", "if SINGLE_BEST_ESTIMATE is used, multiply the planning time estimate by this factor", "1.0", Bounds("0", "infinity"));
	parser.add_option<int>("num_reference_nodes", "Number of reference nodes to consider (use 0 for all reference nodes)", "0", Bounds("0", "infinity"));
	parser.add_option<int>("seed", "Set to -1 (default) to use an arbitrary seed, otherwise initialize the random generator with the given seed.", "-1");
	parser.add_option<bool>("reset_expansion_delay", "Reset expansion delay between search iterations.", "false");
    SearchEngine::add_options_to_parser(parser);
	auto opts = parser.parse();
	opts.set("replan_strategy", 3);
	opts.set("latency", -1);
	return _create_search_engine(parser, std::move(opts));
}

static shared_ptr<SearchEngine> _parse_stop(OptionParser &parser) {
    parser.document_synopsis("Online re-planning search (stop when new goals are available)", "");
	parser.add_option<ParseTree>("eval", "evaluator for h-value used in subsearch");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();
	opts.set("replan_strategy", 0);
	// dummy values for the remaining options
	opts.set("moving_average_size", 0);
	opts.set("reference_node_sampling_method", 0);
	opts.set("single_best_estimate_factor", 1.);
	opts.set("num_reference_nodes", 0);
	opts.set("seed", 0);
	opts.set("reset_expansion_delay", false);
	opts.set("latency", -1);
	return _create_search_engine(parser, std::move(opts));
}

static shared_ptr<SearchEngine> _parse_finish(OptionParser &parser) {
	parser.document_synopsis("Online re-planning search (finish execution when new goals are available)", "");
	parser.add_option<ParseTree>("eval", "evaluator for h-value used in subsearch");
	SearchEngine::add_options_to_parser(parser);
	Options opts = parser.parse();
	opts.set("replan_strategy", 1);
	// dummy values for the remaining options
	opts.set("moving_average_size", 0);
	opts.set("reference_node_sampling_method", 0);
	opts.set("single_best_estimate_factor", 1.);
	opts.set("num_reference_nodes", 0);
	opts.set("seed", 0);
	opts.set("reset_expansion_delay", false);
	opts.set("latency", -1);
	return _create_search_engine(parser, std::move(opts));
}

static shared_ptr<SearchEngine> _parse_fixed_latency(OptionParser &parser) {
	parser.document_synopsis("Online re-planning search (fixed latency)", "");
	parser.add_option<ParseTree>("eval", "evaluator for h-value used in subsearch");
	parser.add_option<int>("latency", "constant latency");
	SearchEngine::add_options_to_parser(parser);
	Options opts = parser.parse();
	opts.set("replan_strategy", 2);
	opts.set("reference_node_sampling_method", 4);
	// dummy values for the remaining options
	opts.set("moving_average_size", 0);
	opts.set("single_best_estimate_factor", 1.);
	opts.set("num_reference_nodes", 0);
	opts.set("seed", 0);
	opts.set("reset_expansion_delay", false);
	return _create_search_engine(parser, std::move(opts));
}

static shared_ptr<SearchEngine> _parse_all(OptionParser &parser) {
	parser.document_synopsis("Online re-planning search (simulate search for all deviation points)", "");
	parser.add_option<ParseTree>("eval", "evaluator for h-value used in subsearch");
	SearchEngine::add_options_to_parser(parser);
	Options opts = parser.parse();
	opts.set("replan_strategy", 4);
	// dummy values for the remaining options
	opts.set("moving_average_size", 0);
	opts.set("reference_node_sampling_method", 0);
	opts.set("single_best_estimate_factor", 1.);
	opts.set("num_reference_nodes", 0);
	opts.set("seed", 0);
	opts.set("reset_expansion_delay", false);
	opts.set("latency", -1);
	return _create_search_engine(parser, std::move(opts));
}

static Plugin<SearchEngine> _plugin("online", _parse);
static Plugin<SearchEngine> _plugin_pruning("online_pruning", _parse_pruning);
static Plugin<SearchEngine> _plugin_stop("online_stop", _parse_stop);
static Plugin<SearchEngine> _plugin_finish("online_finish", _parse_finish);
static Plugin<SearchEngine> _plugin_fixed_latency("online_fixed_latency", _parse_fixed_latency);
static Plugin<SearchEngine> _plugin_all("online_all", _parse_all);
}
