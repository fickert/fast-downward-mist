#! /usr/bin/env python

from __future__ import print_function

import argparse
import json
import random
import re
import os
import shutil


argparser = argparse.ArgumentParser(description='Split goals of PDDL problem files into smaller chunks.')
argparser.add_argument('paths', help='directory containing PDDL problem files', nargs='+')
argparser.add_argument('-o', '--out', help='output directory', default='instances-split')
subgoal_size_group = argparser.add_mutually_exclusive_group(required=True)
subgoal_size_group.add_argument('-s', '--subgoal_size', help='size of the resulting partial goals after splitting', type=int)
subgoal_size_group.add_argument('-n', '--num_subgoals', help='number of equally sized subgoals to split into', type=int)
argparser.add_argument('--seed', help='random seed used for shuffling the goals', type=int)
subgoal_availability_group = argparser.add_mutually_exclusive_group(required=True)
subgoal_availability_group.add_argument('-r', '--remove', help='remove all but the first subgoal', action='store_true')
subgoal_availability_group.add_argument('-a', '--subgoal_availability', help='subgoal availability list', type=int, nargs='+')
subgoal_availability_group.add_argument('-i', '--subgoal_interval', help='subgoal availability interval', type=int)
subgoal_availability_group.add_argument('-p', '--subgoal_properties', help='read subgoal availability interval from properties', type=argparse.FileType())
argparser.add_argument('-f', '--subgoal_expansions_factor', help='if subgoal_properties is used, set online goals at intervals of this factor of the number of expansions', type=float)

args = argparser.parse_args()

assert(all(os.path.exists(path) and os.path.isdir(path) for path in args.paths))
assert(args.subgoal_properties is None or args.subgoal_expansions_factor is not None)


def read_goals(content, and_index, shuffle=True):
    begin_index = and_index + 4
    goals_match = re.match('\s*(\([^)]+\)\s*)+\s*\)\s*\)', content[begin_index:])
    assert(goals_match)
    assert(goals_match.start() == 0)
    end_index = begin_index + goals_match.end()
    goals = re.findall('\([^)]+\)', content[begin_index:end_index])

    if shuffle:
        if args.seed is not None:
            random.seed(args.seed)
        random.shuffle(goals)

    return goals, end_index


def split_goals(goals):
    if args.subgoal_size is not None:
        return [goals[i:min(i + args.subgoal_size, len(goals))] for i in range(0, len(goals), args.subgoal_size)]
    else:
        assert(args.num_subgoals)
        if args.num_subgoals > len(goals):
            return [[goal] for goal in goals]
        min_subgoal_size = len(goals) // args.num_subgoals
        extra_size = len(goals) % args.num_subgoals
        subgoal_sizes = [min_subgoal_size for _ in range(0, args.num_subgoals - extra_size)] + [min_subgoal_size + 1 for _ in range(0, extra_size)]
        assert(len(subgoal_sizes) == args.num_subgoals)
        assert(sum(subgoal_sizes) == len(goals))
        random.shuffle(subgoal_sizes)
        subgoals = []
        used_subgoals = 0
        for subgoal_size in subgoal_sizes:
            subgoals.append(goals[used_subgoals:used_subgoals + subgoal_size])
            used_subgoals += subgoal_size
        assert(len(subgoals) == len(subgoal_sizes))
        assert(all(len(subgoals[i]) == subgoal_sizes[i] for i in range(0, len(subgoals))))
        return subgoals


def parse_properties(properties_file):
    properties = json.load(properties_file)
    assert(isinstance(properties, dict))
    expansions = {}
    for run, attributes in properties.items():
        if 'expansions' in attributes:
            assert(attributes['expansions'] > 0)
            expansions['{}:{}'.format(attributes['domain'], attributes['problem'])] = attributes['expansions']
    return expansions


def get_subgoal_availability(subgoal_index, domain, problem, expansions):
    if args.subgoal_interval is not None:
        return args.subgoal_interval * subgoal_index
    elif args.subgoal_properties is not None:
        assert(expansions)
        assert('{}:{}'.format(domain, problem) in expansions)
        return int(round(args.subgoal_expansions_factor * expansions['{}:{}'.format(domain, problem)] * subgoal_index))
    else:
        assert(args.subgoal_availability is not None)
        assert(len(args.subgoal_availability) == len(subgoals) - 1)
        availability = args.subgoal_availability[subgoal_index - 1]


if args.subgoal_properties:
    print('Reading properties file...')
expansions = parse_properties(args.subgoal_properties) if args.subgoal_properties else None

for path in [os.path.normpath(p) for p in args.paths]:
    print('Processing path {}...'.format(path))
    domain = os.path.basename(path)
    output_path = os.path.join(args.out, domain)
    # create output directory if it doesn't exist
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    for file in os.listdir(path):
        if not os.path.isfile(os.path.join(path, file)) or not file.endswith('.pddl'):
            continue

        if 'domain' in file:
            shutil.copy(os.path.join(path, file), output_path)
            continue

        if args.subgoal_properties and '{}:{}'.format(domain, file) not in expansions:
            print('  No data in properties for problem file {}, skipped.'.format(file))
            continue

        print('  Processing problem file {}...'.format(file), end='')
        with open(os.path.join(path, file), 'r') as f:
            content = f.read()

        try:
            goal_index = content.index('(:goal')
            and_index = content.index('(and', goal_index)
            goals, end_index = read_goals(content, and_index)
            subgoals = split_goals(goals)

            with open(os.path.join(output_path, file), 'w') as f:
                # write initial goal
                f.write(content[:and_index + 4])
                f.write('\n')
                for goal in subgoals[0]:
                    f.write('\t{}\n'.format(goal))
                f.write(')\n)\n')

                if not args.remove:
                    # write online goals
                    f.write('(:onlinegoals\n')
                    for i in range(1, len(subgoals)):
                        f.write('(and\n')
                        for goal in subgoals[i]:
                            f.write('\t{}\n'.format(goal))
                        availability = get_subgoal_availability(i, domain, file, expansions)
                        assert(availability > 0)
                        assert(i == 1 or availability > get_subgoal_availability(i - 1, domain, file, expansions))
                        f.write('\t({:d})\n'.format(availability))
                    f.write(')\n') # end of online goals block

                f.write(content[end_index:]) # remaining content after goal block

            print(' done!')

        except ValueError as e:
            print('    WARNING: unexpected file format in file {}, skipped. Error message:'.format(file))
            print('    {}'.format(e))
            continue
