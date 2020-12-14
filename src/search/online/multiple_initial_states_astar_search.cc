#include "multiple_initial_states_astar_search.h"

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

auto MultipleInitialStateAStarSearch::initialize_reference_nodes_lookup(const std::vector<std::pair<StateID, int>> &reference_nodes) -> std::unordered_map<StateID, int> {
	auto reference_nodes_lookup = std::unordered_map<StateID, int>();
	for (const auto &[reference_state_id, arrival_time] : reference_nodes)
		reference_nodes_lookup[reference_state_id] = arrival_time;
	return reference_nodes_lookup;
}

auto MultipleInitialStateAStarSearch::create_open_list_factory(const options::Options &opts) const -> std::unique_ptr<OpenListFactory> {
	const auto h_eval = opts.get<std::shared_ptr<Evaluator>>("eval");
	const auto g_eval = std::make_shared<g_evaluator::GEvaluator>();
	const auto f_evaluators = std::vector<std::shared_ptr<Evaluator>>{ h_eval, g_eval };
	const auto f_eval = std::make_shared<sum_evaluator::SumEvaluator>(f_evaluators);
	const auto tie_breaking_evals = std::vector<std::shared_ptr<Evaluator>>{ f_eval, h_eval };

	auto open_list_options = options::Options();
	open_list_options.set("evals", tie_breaking_evals);
	open_list_options.set("pref_only", false);
	open_list_options.set("unsafe_pruning", false);
	return std::make_unique<tiebreaking_open_list::TieBreakingOpenListFactory>(open_list_options);
}

auto MultipleInitialStateAStarSearch::initialize_next_action(StateRegistry &state_registry, TaskProxy task_proxy, StateID initial_state_id, const std::vector<std::pair<OperatorID, int>> &planned_execution, const std::vector<std::pair<StateID, int>> &reference_nodes) -> next_action_t {
	auto state = state_registry.lookup_state(initial_state_id);
	auto next_action = next_action_t();
	auto execution_it = std::cbegin(planned_execution);
	for (const auto &[state_id, time] : reference_nodes) {
		while (execution_it != std::cend(planned_execution) && state_id != state.get_id()) {
			state = state_registry.get_successor_state(state, task_proxy.get_operators()[execution_it->first]);
			++execution_it;
		}
		next_action.push_back(execution_it);
	}
	return next_action;
}

MultipleInitialStateAStarSearch::MultipleInitialStateAStarSearch(const options::Options &opts,
                                                                 std::shared_ptr<StateRegistry> state_registry,
                                                                 std::shared_ptr<AbstractTask> task,
                                                                 StateID initial_state_id,
                                                                 std::vector<std::pair<StateID, int>> &&reference_nodes,
                                                                 std::vector<std::pair<OperatorID, int>> &&planned_execution,
                                                                 int moving_average_size,
                                                                 double initial_avg_expansion_delay_value,
                                                                 int last_expansions)
	: SearchEngine(opts, state_registry, task, true),
	  reopen_closed_nodes(opts.get<bool>("reopen_closed")),
	  reference_nodes(std::move(reference_nodes)),
	  reference_nodes_lookup(initialize_reference_nodes_lookup(this->reference_nodes)),
	  planned_execution(std::move(planned_execution)),
	  next_action(initialize_next_action(*state_registry, task_proxy, initial_state_id, this->planned_execution, this->reference_nodes)),
	  expansions(this->reference_nodes.size(), 0),
	  open_list_insertion_time(this->reference_nodes.size()),
	  d_eval(opts.get<std::shared_ptr<Evaluator>>("d_eval")),
	  plan_reference_node_index(static_cast<std::size_t>(-1)) {
	assert(moving_average_size >= 0);
	if (moving_average_size == std::numeric_limits<int>::max())
		moving_average_size = 0;
	if (last_expansions != 0)
		expansion_delay = std::make_unique<ExpansionDelay>(statistics, moving_average_size, initial_avg_expansion_delay_value, last_expansions);
	else
		expansion_delay = std::make_unique<ExpansionDelay>(statistics, moving_average_size);
	auto open_list_factory = create_open_list_factory(opts);

	open_lists.reserve(this->reference_nodes.size());
	search_spaces.reserve(this->reference_nodes.size());
	for (auto i = 0u; i < this->reference_nodes.size(); ++i) {
		open_lists.push_back(open_list_factory->create_state_open_list());
		search_spaces.push_back(std::make_unique<SearchSpace>(*this->state_registry));
	}
	min_distance = std::vector<int>(this->reference_nodes.size(), std::numeric_limits<int>::max());
}

void MultipleInitialStateAStarSearch::initialize() {
    cout << "Conducting A* search with multiple initial states (" << reference_nodes.size() << "),"
         << (reopen_closed_nodes ? " with" : " without")
         << " reopening closed nodes, (real) bound = " << bound
         << endl;
    assert(!open_lists.empty());

	assert(reference_nodes.size() == open_lists.size());
	for (auto i = static_cast<std::size_t>(0); i < reference_nodes.size(); ++i) {
		auto &open_list = open_lists[i];
		const auto &reference_state_id = reference_nodes[i].first;
		const auto &state = state_registry->lookup_state(reference_state_id);
		auto eval_context = EvaluationContext(state, 0, true, &statistics);
		statistics.inc_evaluated_states();

		if (open_list->is_dead_end(eval_context))
			continue;
		if (i == 0 && search_progress.check_progress(eval_context))
			print_checkpoint_line(0);
		SearchNode node = search_spaces.at(i)->get_node(state);
		node.open_initial();

		open_list->insert(eval_context, reference_state_id);
		open_list_insertion_time[i][reference_state_id] = 0;
		min_distance[i] = eval_context.get_evaluator_value(d_eval.get());
	}
}

SearchStatus MultipleInitialStateAStarSearch::step() {
    auto [node, success, reference_node_index] = fetch_next_node();
    if (!success)
        return FAILED;

    GlobalState s = node.get_state();
    if (check_goal_and_set_plan(s, *search_spaces[reference_node_index])) {
		plan_reference_node_index = reference_node_index;
        return SOLVED;
	}

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(s, applicable_ops);

    // This evaluates the expanded state (again) to get preferred ops
    EvaluationContext eval_context(s, node.get_g(), false, &statistics, true);

	// Get open list of reference node with child @link node
	auto &open_list = open_lists.at(reference_node_index);
	auto &search_space = search_spaces.at(reference_node_index);

    for (OperatorID op_id : applicable_ops) {
        OperatorProxy op = task_proxy.get_operators()[op_id];
        if ((node.get_real_g() + op.get_cost()) >= bound)
            continue;

        GlobalState succ_state = state_registry->get_successor_state(s, op);
        statistics.inc_generated();

        SearchNode succ_node = search_space->get_node(succ_state);

        // Previously encountered dead end. Don't re-evaluate.
        if (succ_node.is_dead_end())
            continue;

        if (prune_state(reference_node_index, succ_state)) {
            if (succ_node.is_new())
                succ_node.open(node, op, get_adjusted_cost(op));
            if (succ_node.is_open())
                succ_node.close();
            continue;
        }

        if (succ_node.is_new()) {
            // We have not seen this state before.
            // Evaluate and create a new node.

            // Careful: succ_node.get_g() is not available here yet,
            // hence the stupid computation of succ_g.
            // TODO: Make this less fragile.
            int succ_g = node.get_g() + get_adjusted_cost(op);

            EvaluationContext eval_context(
                succ_state, succ_g, false, &statistics);
            statistics.inc_evaluated_states();

            if (open_list->is_dead_end(eval_context)) {
                succ_node.mark_as_dead_end();
                statistics.inc_dead_ends();
                continue;
            }
            succ_node.open(node, op, get_adjusted_cost(op));

            open_list->insert(eval_context, succ_state.get_id());
			open_list_insertion_time[reference_node_index][succ_state.get_id()] = expansions[reference_node_index];
			assert(!eval_context.is_evaluator_value_infinite(d_eval.get()));
			min_distance[reference_node_index] = std::min(min_distance[reference_node_index], eval_context.get_evaluator_value(d_eval.get()));
            if (search_progress.check_progress(eval_context)) {
                print_checkpoint_line(succ_node.get_g());
                reward_progress();
            }
        } else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(op)) {
            // We found a new cheapest path to an open or closed state.
            if (reopen_closed_nodes) {
                if (succ_node.is_closed()) {
                    /*
                      TODO: It would be nice if we had a way to test
                      that reopening is expected behaviour, i.e., exit
                      with an error when this is something where
                      reopening should not occur (e.g. A* with a
                      consistent heuristic).
                    */
                    statistics.inc_reopened();
                }
                succ_node.reopen(node, op, get_adjusted_cost(op));
				open_list_insertion_time[reference_node_index][succ_state.get_id()] = expansions[reference_node_index];

                EvaluationContext eval_context(
                    succ_state, succ_node.get_g(), false, &statistics);

                /*
                  Note: our old code used to retrieve the h value from
                  the search node here. Our new code recomputes it as
                  necessary, thus avoiding the incredible ugliness of
                  the old "set_evaluator_value" approach, which also
                  did not generalize properly to settings with more
                  than one evaluator.

                  Reopening should not happen all that frequently, so
                  the performance impact of this is hopefully not that
                  large. In the medium term, we want the evaluators to
                  remember evaluator values for states themselves if
                  desired by the user, so that such recomputations
                  will just involve a look-up by the Evaluator object
                  rather than a recomputation of the evaluator value
                  from scratch.
                */
                open_list->insert(eval_context, succ_state.get_id());
            } else {
                // If we do not reopen closed nodes, we just update the parent pointers.
                // Note that this could cause an incompatibility between
                // the g-value and the actual path that is traced back.
                succ_node.update_parent(node, op, get_adjusted_cost(op));
            }
        }
    }

    return IN_PROGRESS;
}

tuple<SearchNode, bool, std::size_t> MultipleInitialStateAStarSearch::fetch_next_node() {
    /* TODO: The bulk of this code deals with multi-path dependence,
       which is a bit unfortunate since that is a special case that
       makes the common case look more complicated than it would need
       to be. We could refactor this by implementing multi-path
       dependence as a separate search algorithm that wraps the "usual"
       search algorithm and adds the extra processing in the desired
       places. I think this would lead to much cleaner code. */

    while (true) {
		const auto best_index = return_best_index(open_lists);
		auto &open_list = open_lists.at(best_index);

        if (open_list->empty()) {
            cout << "Completely explored state space -- no solution!" << endl;
            // HACK! HACK! we do this because SearchNode has no default/copy constructor
            const GlobalState &initial_state = state_registry->get_initial_state();
            SearchNode dummy_node = search_spaces.at(best_index)->get_node(initial_state);
            return {dummy_node, false, best_index};
        }
        StateID id = open_list->remove_min();
        // TODO is there a way we can avoid creating the state here and then
        //      recreate it outside of this function with node.get_state()?
        //      One way would be to store GlobalState objects inside SearchNodes
        //      instead of StateIDs
        GlobalState s = state_registry->lookup_state(id);
        SearchNode node = search_spaces.at(best_index)->get_node(s);

        if (node.is_closed())
            continue;

        assert(!node.is_dead_end());
        node.close();

        // prune this node if it is a later reference node with the same or greater g-value
        if (node.get_state_id() != reference_nodes[best_index].first) {
            const auto lookup_result = reference_nodes_lookup.find(node.get_state_id());
            if (lookup_result != std::end(reference_nodes_lookup) && reference_nodes[best_index].second + node.get_g() * task->get_execution_time() >= lookup_result->second)
                continue;
        }

        assert(!node.is_dead_end());
        statistics.inc_expanded();
		++expansions[best_index];
		assert(expansions[best_index] > open_list_insertion_time[best_index][id]);
		expansion_delay->update_expansion_delay(expansions[best_index] - open_list_insertion_time[best_index][id]);
		return {node, true, best_index};
    }
}

auto MultipleInitialStateAStarSearch::check_goal_and_set_plan(const GlobalState &state, const SearchSpace &search_space) -> bool {
	if (task_properties::is_goal_state(task_proxy, state)) {
		cout << "Solution found!" << endl;
		Plan plan;
		search_space.trace_path(state, plan);
		set_plan(plan);
		return true;
	}
	return false;
}

void MultipleInitialStateAStarSearch::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
	for (auto &open_list : open_lists)
		open_list->boost_preferred();
}

void MultipleInitialStateAStarSearch::print_checkpoint_line(int g) const {
    cout << "[g=" << g << ", ";
    statistics.print_basic_statistics();
    cout << "]" << endl;
}

void MultipleInitialStateAStarSearch::print_statistics() const {
    statistics.print_detailed_statistics();
	for (auto &search_space : search_spaces)
		search_space->print_statistics();
}

void MultipleInitialStateAStarSearch::dump_search_space() const {
    search_space.dump(task_proxy);
}

}
