#!/usr/bin/env python

from six.moves.queue import Queue  # pylint: disable=relative-import

import math
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      ActorSource,
                                                                      ActorSink,
                                                                      WaypointFollower,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, DrivenDistanceTest, MaxVelocityTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class CFFNudge(BasicScenario):

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=120):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._drivenDistanceM = 150

        # Timeout of scenario in seconds
        self.timeout = timeout

        super(CFFNudge, self).__init__(
            "CFFNudge",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        # add actors from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=True)

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        # sequence.add_child(Idle())

        driveDistance = DriveDistance(
            self.ego_vehicles[0],
            self._drivenDistanceM,
            name="DriveDistance")

        # Build behavior tree
        sequence.add_child(driveDistance)

        return sequence

    def _create_test_criteria(self):
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        driven_distance_criterion = DrivenDistanceTest(self.ego_vehicles[0], self._drivenDistanceM)

        criteria.append(collision_criterion)
        criteria.append(driven_distance_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
