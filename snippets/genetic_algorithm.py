import logging
import math
import numpy as np
from pymoo.algorithms.base.genetic import GeneticAlgorithm
from pymoo.core.infill import InfillCriterion
from pymoo.core.mixed import MixedVariableDuplicateElimination, MixedVariableSampling
from pymoo.core.individual import Individual
from pymoo.core.population import Population
from pymoo.core.problem import Problem
from pymoo.core.survival import Survival
from pymoo.util.randomized_argsort import randomized_argsort
from pymoo.operators.survival.rank_and_crowding.metrics import get_crowding_function
from pymoo.util.nds.non_dominated_sorting import NonDominatedSorting
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.core.variable import Real, Integer, Choice
from pymoo.operators.selection.rnd import RandomSelection
from pymoo.operators.selection.tournament import compare, TournamentSelection
from pymoo.util.dominator import Dominator
from pymoo.operators.repair.rounding import RoundingRepair

logger = logging.getLogger(__name__)

def binary_tournament(pop, P, algorithm, **kwargs):
    logger.info("Tournament P: %s", str(P))
    n_tournaments, n_parents = P.shape

    if n_parents != 2:
        raise ValueError("Only implemented for binary tournament!")

    S = np.full(n_tournaments, np.nan)

    for i in range(n_tournaments):

        a, b = P[i, 0], P[i, 1]
        a_cv, a_f, b_cv, b_f = pop[a].CV[0], pop[a].F, pop[b].CV[0], pop[b].F
        rank_a, cd_a = pop[a].get("rank", "crowding")
        rank_b, cd_b = pop[b].get("rank", "crowding")
        logger.info("tournament: %d, %f, %d, %f", rank_a, cd_a, rank_b, cd_b)

        # if at least one solution is infeasible
        if a_cv > 0.0 or b_cv > 0.0:
            S[i] = compare(a, a_cv, b, b_cv, method='smaller_is_better', return_random_if_equal=True)

        # both solutions are feasible
        else:
            S[i] = compare(a, rank_a, b, rank_b, method='smaller_is_better')

            # if rank or domination relation didn't make a decision compare by crowding
            if np.isnan(S[i]):
                S[i] = compare(a, cd_a, b, cd_b, method='larger_is_better', return_random_if_equal=True)

    return S[:, None].astype(int, copy=False)

class MixedVariableMating(InfillCriterion):

    def __init__(self,
                 selection=TournamentSelection(func_comp=binary_tournament),
                 crossover=None,
                 mutation=None,
                 repair=None,
                 eliminate_duplicates=True,
                 n_max_iterations=100,
                 **kwargs):

        super().__init__(repair, eliminate_duplicates, n_max_iterations, **kwargs)

        if crossover is None:
            crossover = {
                Real: SBX(eta=15, prob=0.9),
                Integer: SBX(vtype=float, repair=RoundingRepair()),
            }

        if mutation is None:
            mutation = {
                Real: PM(eta=20),
                Integer: PM(vtype=float, repair=RoundingRepair()),
            }

        self.selection = selection
        self.crossover = crossover
        self.mutation = mutation

    def _do(self, problem, pop, n_offsprings, parents=False, **kwargs):
        logger.info("===Population=== \n X: %s \n F: %s", pop.get("X"), pop.get("F"))

        # So far we assume all crossover need the same amount of parents and create the same number of offsprings
        XOVER_N_PARENTS = 2
        XOVER_N_OFFSPRINGS = 2

        # the variables with the concrete information
        vars = problem.vars

        # group all the variables by their types
        vars_by_type = {}
        for k, v in vars.items():
            clazz = type(v)

            if clazz not in vars_by_type:
                vars_by_type[clazz] = []
            vars_by_type[clazz].append(k)

        # # all different recombinations (the choices need to be split because of data types)
        recomb = []
        for clazz, list_of_vars in vars_by_type.items():
            if clazz == Choice:
                for e in list_of_vars:
                    recomb.append((clazz, [e]))
            else:
                recomb.append((clazz, list_of_vars))

        # create an empty population that will be set in each iteration
        off = Population.new(X=[{} for _ in range(n_offsprings)])

        if not parents:
            n_select = math.ceil(n_offsprings / XOVER_N_OFFSPRINGS)
            pop = self.selection(problem, pop, n_select, XOVER_N_PARENTS, **kwargs)
            logger.info("===Parents=== \n X: %s \n F: %s", pop.get("X"), pop.get("F"))

        for clazz, list_of_vars in recomb:

            crossover = self.crossover[clazz]
            assert crossover.n_parents == XOVER_N_PARENTS and crossover.n_offsprings == XOVER_N_OFFSPRINGS

            _parents = [
                [Individual(X=np.array([parent.X[var] for var in list_of_vars], dtype="O" if clazz is Choice else None))
                  for parent in parents]
                for parents in pop
            ]

            _vars = {e: vars[e] for e in list_of_vars}
            _xl = np.array([vars[e].lb if hasattr(vars[e], "lb") else None for e in list_of_vars])
            _xu = np.array([vars[e].ub if hasattr(vars[e], "ub") else None for e in list_of_vars])
            _problem = Problem(vars=_vars, xl=_xl, xu=_xu)

            _off = crossover(_problem, _parents, **kwargs)

            mutation = self.mutation[clazz]
            _off = mutation(_problem, _off, **kwargs)

            for k in range(n_offsprings):
                for i, name in enumerate(list_of_vars):
                    off[k].X[name] = _off[k].X[i]

        logger.info("===Offsprings=== \n X: %s \n F: %s", off.get("X"), off.get("F"))
        return off


class RankAndCrowding(Survival):

    def __init__(self, nds=None, crowding_func="cd"):
        crowding_func_ = get_crowding_function(crowding_func)

        super().__init__(filter_infeasible=True)
        self.nds = nds if nds is not None else NonDominatedSorting()
        self.crowding_func = crowding_func_

    def _do(self,
            problem,
            pop,
            *args,
            n_survive=None,
            **kwargs):

        # get the objective space values and objects
        F = pop.get("F").astype(float, copy=False)

        # the final indices of surviving individuals
        survivors = []

        # do the non-dominated sorting until splitting front
        fronts = self.nds.do(F, n_stop_if_ranked=n_survive)
        logger.info("Fronts: %s", str(fronts))
        for k, front in enumerate(fronts):

            I = np.arange(len(front))

            # current front sorted by crowding distance if splitting
            if len(survivors) + len(I) > n_survive:

                # Define how many will be removed
                n_remove = len(survivors) + len(front) - n_survive

                # re-calculate the crowding distance of the front
                crowding_of_front = \
                    self.crowding_func.do(
                        F[front, :],
                        n_remove=n_remove
                    )

                I = randomized_argsort(crowding_of_front, order='descending', method='numpy')
                I = I[:-n_remove]

            # otherwise take the whole front unsorted
            else:
                # calculate the crowding distance of the front
                crowding_of_front = \
                    self.crowding_func.do(
                        F[front, :],
                        n_remove=0
                    )

            # save rank and crowding in the individual class
            for j, i in enumerate(front):
                pop[i].set("rank", k)
                pop[i].set("crowding", crowding_of_front[j])
                logger.info("set rank and crowding: %d, %f", k, crowding_of_front[j])

            # extend the survivors by all or selected individuals
            survivors.extend(front[I])

        return pop[survivors]

class MixedVariableGA(GeneticAlgorithm):

    def __init__(self,
                 pop_size=50,
                 n_offsprings=None,
                 output=None,
                 sampling=MixedVariableSampling(),
                 mating=None,
                 eliminate_duplicates=MixedVariableDuplicateElimination(),
                 survival=None,
                 **kwargs):
        super().__init__(pop_size=pop_size, n_offsprings=n_offsprings, sampling=sampling, mating=mating,
                         eliminate_duplicates=eliminate_duplicates, output=output, survival=survival, advance_after_initial_infill=True, **kwargs)
