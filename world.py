#!/usr/bin/env python
from gazeboEnacter import GazeboEnacter
import random



class Agent:
    def __init__(self, _hedonist_table):
        """ Creating our agent

        :param _hedonist_table: array composed of 3 arrays, one for each actions : go forward,
                                turn left and turn right. In each array, the first value
                                correspond to a free way and the second one to an obstacle.
        """

        self.hedonist_table = _hedonist_table
        self._action = 0
        self.new_action = 0
        self.anticipated_outcome = 0
        self.anticipation_0 = 0
        self.anticipation_1 = 0
        self.anticipation_2 = 0
        self.ennui = 0
        self.valeur_hedoniste_anticipee = [[0, 0], [0, 0], [0, 0]]
        self.valeur_hedoniste_anticipee_pour_action_0 = 0
        self.valeur_hedoniste_anticipee_pour_action_1 = 0
        self.valeur_hedoniste_anticipee_pour_action_2 = 0

    def change_action(self, action):
        if action == 0:
            new_action = self.turn_choice()
        else:
            new_action = 0
        return new_action

    def increment_ennui(self, action):
        if self.ennui >= 2:
            new_action = self.change_action(action)
            self.ennui = 0
        else:
            self.ennui += 1
            new_action = action
        return new_action

    def turn_choice(self):
        if self.valeur_hedoniste_anticipee[1][0] > self.valeur_hedoniste_anticipee[2][0]:
            new_action = 1
        elif self.valeur_hedoniste_anticipee[1][0] < self.valeur_hedoniste_anticipee[2][0]:
            new_action = 2
        else:
            new_action = random.randint(1, 2)
        return new_action

    def upgraded_action(self, outcome):
        self.valeur_hedoniste_anticipee[self._action][outcome] = self.hedonist_table[self._action][outcome]
        if self._action == 0:
            self.anticipation_0 = outcome
            if self.valeur_hedoniste_anticipee[self._action][outcome] > max(self.valeur_hedoniste_anticipee[1][0],
                                                                            self.valeur_hedoniste_anticipee[1][1]):
                self._action = self.increment_ennui(self._action)
            else:
                self._action = self.change_action(self._action)
                self.ennui = 0
        else:
            self.anticipation_1 = outcome
            if self.valeur_hedoniste_anticipee[self._action][outcome] > max(self.valeur_hedoniste_anticipee[0][0],
                                                                            self.valeur_hedoniste_anticipee[0][1]):
                self._action = self.increment_ennui(self._action)
            else:
                self._action = self.change_action(self._action)
                self.ennui = 0
        return self._action

    def action(self, outcome):
        """ Computing the next action to enact

        :param outcome: the outcome of the last action
        :return: the value of the action.
        """

        # action forward
        if self._action == 0:
            self.anticipation_0 = outcome
            self.valeur_hedoniste_anticipee_pour_action_0 = self.hedonist_table[0][self.anticipation_0]
            if outcome == self.anticipated_outcome and self.valeur_hedoniste_anticipee_pour_action_0 >= self.valeur_hedoniste_anticipee_pour_action_1:
                self.new_action = self._action
            if self.ennui >= 3 or self.valeur_hedoniste_anticipee_pour_action_0 <= self.valeur_hedoniste_anticipee_pour_action_1:
                # compare with previous turn
                if self._action == 0:
                    self.new_action = random.randint(1, 2)
                if self._action == 1 or self._action == 2:
                    self.new_action = 0

        # action turn left
        elif self._action == 1:
            action = 1
            self.anticipation_1 = outcome
            self.valeur_hedoniste_anticipee_pour_action_1 = self.hedonist_table[1][self.anticipation_1]
            if outcome == self.anticipated_outcome and self.valeur_hedoniste_anticipee_pour_action_1 >= self.valeur_hedoniste_anticipee_pour_action_0:
                self.new_action = self._action
            if self.ennui >= 3 or self.valeur_hedoniste_anticipee_pour_action_1 <= self.valeur_hedoniste_anticipee_pour_action_0:
                # compare with previous turn
                if self._action == 0:
                    self.new_action = random.randint(1, 2)
                if self._action == 1 or self._action == 2:
                    self.new_action = 0

        # action turn right
        elif self._action == 2:
            self.anticipation_2 = outcome
            self.valeur_hedoniste_anticipee_pour_action_2 = self.hedonist_table[2][self.anticipation_2]
            if outcome == self.anticipated_outcome and self.valeur_hedoniste_anticipee_pour_action_2 >= self.valeur_hedoniste_anticipee_pour_action_0:
                self.new_action = self._action
            if self.ennui >= 3 or self.valeur_hedoniste_anticipee_pour_action_2 <= self.valeur_hedoniste_anticipee_pour_action_0:
                # compare with previous turn
                if self._action == 0:
                    self.new_action = random.randint(1, 2)
                if self._action == 1 or self._action == 2:
                    self.new_action = 0

        if self._action == self.new_action:
            self.ennui = self.ennui + 1
        else:
            self.ennui = 0
        if self.ennui >= 4:
            self.ennui = 0

        self._action = self.new_action

        return self._action

    def anticipation(self):
        """ Computing the anticipated outcome from the latest action """

        if self._action == 0:
            self.anticipated_outcome = self.anticipation_0
        else:
            self.anticipated_outcome = self.anticipation_1
        return self.anticipated_outcome

    def satisfaction(self, new_outcome):
        """ Computing a tuple representing the agent's satisfaction after the last interaction

        :param new_outcome: the most recent outcome
        :type new_outcome: int
        :return: anticipation_satisfaction : returns whether the robot has anticipated the correct outcome or not
                 hedonist_satisfaction : satisfaction of the last action in relation to the hedonist table
                 ennui : returns whether the robot is bored or not
        :rtype: bool, array of array, bool
        """

        # True if the anticipation was correct
        anticipation_satisfaction = (self.anticipated_outcome == new_outcome)
        # The value of the enacted interaction
        hedonist_satisfaction = self.hedonist_table[self._action][new_outcome]
        ennui = False
        if self.ennui == 2:
            ennui = True
        else:
            ennui = False
        return anticipation_satisfaction, hedonist_satisfaction, ennui


def world(agent, environment):
    """ The main loop controlling the interaction of the agent with the environment

    :param agent:
    :type agent:
    :param environment:
    :type environment:
    """

    outcome = 0
    for i in range(20):
        action = agent.upgraded_action(outcome)
        outcome = environment.outcome(action)
        if action == 1:
            nom_action = "turn left"
        elif action == 2:
            nom_action = "turn right"
        else:
            nom_action = "forward"

        if outcome == 1:
            nom_outcome = "obstacle"
        else:
            nom_outcome = "no obstacle"

        if agent.anticipation() == 1:
            nom_anticipation = "obstacle"
        else:
            nom_anticipation = "no obstacle"

        print(" Action: " + nom_action + ", Anticipation: " + nom_anticipation + ", Outcome: " + nom_outcome
              + ", Satisfaction: " + str(agent.satisfaction(outcome)) + "\n")


hedonist_table = [[1, -3], [-2, -2], [-2, -2]]
a = Agent(hedonist_table)
# e = Environment1()
# e = Environment2()
e = GazeboEnacter()

world(a, e)
