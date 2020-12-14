#include "modified_goals_task.h"

#include "root_task.h"
#include "../options/option_parser.h"
#include "../options/plugin.h"

using namespace std;

namespace extra_tasks {
ModifiedGoalsTask::ModifiedGoalsTask(
    const shared_ptr<AbstractTask> &parent,
    vector<FactPair> &&goals)
    : DelegatingTask(parent),
      goals(move(goals)) {
}

int ModifiedGoalsTask::get_num_goals() const {
    return goals.size();
}

FactPair ModifiedGoalsTask::get_goal_fact(int index) const {
    return goals[index];
}

static shared_ptr<AbstractTask> _parse(options::OptionParser &parser) {
	parser.add_list_option<int>("variables");
	parser.add_list_option<int>("values");
	options::Options opts = parser.parse();
	if (parser.dry_run()) {
		return nullptr;
	} else {
		auto goals = std::vector<FactPair>();
		auto variables = opts.get_list<int>("variables");
		auto values = opts.get_list<int>("values");
		assert(variables.size() == values.size());
		for (auto i = 0u; i < variables.size(); ++i)
			goals.emplace_back(variables[i], values[i]);
		return std::make_shared<ModifiedGoalsTask>(tasks::g_root_task, std::move(goals));
	}
}

static options::Plugin<AbstractTask> _plugin("modified_goals", _parse);
}
