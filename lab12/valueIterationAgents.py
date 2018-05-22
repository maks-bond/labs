# valueIterationAgents.py
# -----------------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


import mdp, util

from learningAgents import ValueEstimationAgent

class ValueIterationAgent(ValueEstimationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A ValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100):
        """
          Your value iteration agent should take an mdp on
          construction, run the indicated number of iterations
          and then act according to the resulting policy.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state, action, nextState)
              mdp.isTerminal(state)
        """
        self.mdp = mdp
        self.discount = discount
        self.iterations = iterations
        self.values = util.Counter() # A Counter is a dict with default 0
        self.qvalues = util.Counter()
        # Write value iteration code here
        "*** YOUR CODE HERE ***"

        #print(iterations)
        cur_discount = discount
        new_values = util.Counter()
        for it in range(iterations+1):
            self.values = new_values
            new_values = util.Counter()
            for state in mdp.getStates():
                best_q_value = -10000000
                # TODO: handle terminal states
                for action in mdp.getPossibleActions(state):
                    self.qvalues[(state, action)] = 0
                    for state_prob in mdp.getTransitionStatesAndProbs(state, action):
                        next_state = state_prob[0]
                        prob = state_prob[1]
                        # print("Current state: " + str(state))
                        # print("Action: " + str(action))
                        # print("Next state: " + str(next_state))
                        # print("Discount: " + str(cur_discount))
                        # print("Reward: " + str(self.mdp.getReward(state, action, next_state)))
                        # print("Prob: " + str(prob))
                        # print("Value: " + str(self.values[next_state]))

                        self.qvalues[(state, action)] += prob*(mdp.getReward(state, action, next_state)+cur_discount*self.values[next_state])

                        # print("Q-Value: " + str(self.qvalues[(state, action)]))
                        # print("==========\n")

                    if self.qvalues[(state, action)] > best_q_value:
                        best_q_value = self.qvalues[(state, action)]

                if self.mdp.isTerminal(state) == False:
                    new_values[state] = best_q_value
                else:
                    new_values[state] = 0
            #self.values = new_values
            #print(self.values)
            #print("\n")
            #cur_discount *= discount

    def getValue(self, state):
        """
          Return the value of the state (computed in __init__).
        """
        return self.values[state]


    def computeQValueFromValues(self, state, action):
        """
          Compute the Q-value of action in state from the
          value function stored in self.values.
        """
        "*** YOUR CODE HERE ***"
        #print(self.qvalues[(state, action)])
        return self.qvalues[(state, action)]

    def computeActionFromValues(self, state):
        """
          The policy is the best action in the given state
          according to the values currently stored in self.values.

          You may break ties any way you see fit.  Note that if
          there are no legal actions, which is the case at the
          terminal state, you should return None.
        """
        "*** YOUR CODE HERE ***"
        if self.mdp.isTerminal(state):
            return None

        best_action = None
        best_q_value = -10000000
        for action in self.mdp.getPossibleActions(state):
            q_value = self.qvalues[(state, action)]
            if q_value > best_q_value:
                best_q_value = q_value
                best_action = action

        return best_action

    def getPolicy(self, state):
        return self.computeActionFromValues(state)

    def getAction(self, state):
        "Returns the policy at the state (no exploration)."
        return self.computeActionFromValues(state)

    def getQValue(self, state, action):
        return self.computeQValueFromValues(state, action)
