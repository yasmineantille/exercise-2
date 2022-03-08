#
# This file is based on pyperplan
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>
#

"""
Classes for representing a STRIPS planning task
"""


class Operator:

    def __init__(self, name, preconditions, add_effects, del_effects):
        """
        The constructor of the class Operator. 

        In a STRIPS planning task, an Operator is an action that can be applied
        to change the state during the execution of a plan.

        In this class all sets are implemented as Python frozensets.
        See more here: https://www.programiz.com/python-programming/methods/built-in/frozenset
        
        @param name The name of the operator (e.g. "action1 operand1 operand2" 
               is the str name of an operator for the action action1 and the
               grounded operands operand1 and operand2)
        @param preconditions The preconditions (the facts) that need to hold 
               at a given state for the operator to be applicable in state
        @param add_effects the facts that become true in state after applying 
               the operator
        @param del_effects the facts that become false in state after applying
               the operator
        """
        self.name = name
        self.preconditions = frozenset(preconditions)
        self.add_effects = frozenset(add_effects)
        self.del_effects = frozenset(del_effects)

    # ---- Step 1 ----
    # Implement the method
    def applicable(self, state):
        """
        Operators are applicable when their set of preconditions is a subset
        of the facts that are true in "state".

        @return True if the operator's preconditions is a subset of the state,
                False otherwise
        """
        is_subset = self.preconditions.issubset(state)
        return is_subset

    # ---- Step 2 ----
    # Implement the method
    def apply(self, state):
        """
        Applying an operator means removing the facts that are made false
        by the operator from the set of true facts in state and adding
        the facts made true.

        Note that therefore it is possible to have operands that make a
        fact both false and true. This results in the fact being true
        at the end.

        @param state The state that the operator should be applied to
        @return A new state (set of facts) after the application of the
                operator
        """
        new_state = state.difference(self.del_effects).union(self.add_effects)
        return new_state

    def __eq__(self, other):
        return (
            self.name == other.name
            and self.preconditions == other.preconditions
            and self.add_effects == other.add_effects
            and self.del_effects == other.del_effects
        )

    def __hash__(self):
        return hash((self.name, self.preconditions, self.add_effects, self.del_effects))

    def __str__(self):
        s = "%s\n" % self.name
        for group, facts in [
            ("PRE", self.preconditions),
            ("ADD", self.add_effects),
            ("DEL", self.del_effects),
        ]:
            for fact in facts:
                s += f"  {group}: {fact}\n"
        return s

    def __repr__(self):
        return "<Op %s>" % self.name


class Task:
    """
    A STRIPS planning task
    """

    def __init__(self, name, facts, initial_state, goals, operators):
        """
        @param name The task's name (i.e. the name of the defined problem)
        @param facts A set of all the fact names that are valid in the domain
        @param initial_state A set of fact names that are true at the beginning
        @param goals A set of fact names that must be true to solve the problem
        @param operators A set of operator instances for the domain
        """
        self.name = name
        self.facts = facts
        self.initial_state = initial_state
        self.goals = goals
        self.operators = operators

    # ---- Step 3 ----
    # Implement the method
    def goal_reached(self, state):
        """
        The goal has been reached if all facts that are true in "goals"
        are true in "state".

        @param state A state
        @return True if all the goals are reached, False otherwise
        """
        goal_reached = self.goals.issubset(state)
        return goal_reached

    # ---- Step 4 ----
    # Implement the method
    def get_successor_states(self, state):
        """
        Return all the possible next states that can be reached after "state", and
        the operators that need to be applied for reaching the next states.

        For every operator of the Task instance, if an operator is applicable at the
        current "state", store in a list the pair of the operator, and the new (next) state
        that holds when applying the operator in "state".
        
        @param state A state 
        @return A list with (op, new_state) pairs where "op" is the applicable
        operator and "new_state" the state that results when "op" is applied
        in state "state".
        """
        operators = []
        for op in self.operators:
            if op.applicable(state):
                operators.append((op, op.apply(state)))
        return operators

    def __str__(self):
        s = "Task {0}\n  Vars:  {1}\n  Init:  {2}\n  Goals: {3}\n  Ops:   {4}"
        return s.format(
            self.name,
            ", ".join(self.facts),
            self.initial_state,
            self.goals,
            "\n".join(map(repr, self.operators)),
        )

    def __repr__(self):
        string = "<Task {0}, vars: {1}, operators: {2}>"
        return string.format(self.name, len(self.facts), len(self.operators))
