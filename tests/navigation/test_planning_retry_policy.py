"""Deployment XML must use the tested stop/replan scheduler."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from fairino_slam.deployment.build_navigation import build

ROOT = Path(__file__).resolve().parents[2] / 'config' / 'behavior_trees'


class PlanningRetryPolicyTests(unittest.TestCase):
    def test_both_trees_use_scheduler_with_unchanged_goal_and_single_follower(self):
        for name, action, goal in (
                ('navigate_to_pose.xml', 'ComputePathToPose', 'goal'),
                ('navigate_through_poses.xml', 'ComputePathThroughPoses', 'goals')):
            with self.subTest(tree=name):
                root = ET.parse(ROOT/name).getroot()
                scheduler = root.find('./BehaviorTree/ReplanUntilClear')
                self.assertIsNotNone(scheduler)
                self.assertEqual([child.tag for child in scheduler], [action, 'FollowPath'])
                self.assertEqual(scheduler[0].attrib,
                                 {goal: '{'+goal+'}', 'path': '{path}', 'planner_id': 'GridBased'})
                self.assertEqual(scheduler[1].attrib['path'], '{path}')
                self.assertEqual(scheduler[1].attrib['goal_checker_id'], 'goal_checker')
                self.assertEqual(len(root.findall('.//FollowPath')), 1)
                self.assertLessEqual({n.tag for n in root.iter()},
                                    {'root', 'BehaviorTree', 'ReplanUntilClear', action, 'FollowPath'})

    def test_scheduler_plugin_is_in_generated_configuration(self):
        self.assertIn('fairino_replan_bt_node',
                      build()['bt_navigator']['ros__parameters']['plugin_lib_names'])
