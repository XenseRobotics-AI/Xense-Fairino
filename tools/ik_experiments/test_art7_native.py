"""Independent acceptance tests for the native ART7 constrained IK.

Run from the repository root with the repository Python interpreter. These
tests use independent stdlib FK and the unchanged wrist guard, never robot I/O.
They deliberately do not measure performance; benchmarks must run separately.
"""
import itertools
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(Path(__file__).resolve().parent),
               str(ROOT/'apps/fairino_tcp_probe'), str(ROOT/'apps/fairino_upper_probe')]

import numpy as np

import test_art7_qp_contract as contracts
from tools.ik_experiments.art7_native import Art7NativeIK, solve_box_qp_native
from art7_qp import Art7ConstrainedIK, rotation_log, solve_box_qp
from tcp_core import BOUNDS_DEG, multiply, pose_error, tcp_forward_matrix
from tool_frames import tool_config, for_arm
from wrist_guard import WristGuard, cap


def independent_fk(q, tools, side):
    return np.asarray(multiply(tcp_forward_matrix(np.asarray(q).tolist()), for_arm(tools, side)))


class NativeContractTests(contracts.SolverContractTests):
    """Run the established output contracts against the C++ implementation."""
    def setUp(self):
        self.tools = tool_config('flange')
        self.solver = Art7NativeIK(tools=self.tools, side='left')
        self.guard = WristGuard()

    def test_custom_position_limits_are_enforced_and_seed_is_not_clipped(self):
        # Override the inherited test because its local constructor is Python.
        bounds = np.asarray(BOUNDS_DEG).copy()
        bounds[5:] = [-40., 40.]
        solver = Art7NativeIK(tools=self.tools, bounds_deg=bounds)
        seed = np.radians([45., 75., -100., 85., 135., 39.8, 39.8])
        goal = solver.forward(seed).copy(); goal[0, 3] -= 2.
        result = solver.solve(goal, seed, max_iterations=80)
        q = self.assert_valid_result(result, seed, goal, custom_bounds=bounds)
        self.assertTrue(result['success'], result)
        self.assertGreater(float(np.abs(q[:5]-seed[:5]).max()), math.radians(.01))
        outside = seed.copy(); outside[5] = math.radians(40.1)
        snapshot = outside.copy()
        with self.assertRaises(ValueError):
            solver.solve(solver.forward(outside), outside)
        np.testing.assert_array_equal(outside, snapshot)

    def test_invalid_time_state_and_bounds_are_explicitly_rejected(self):
        goal = self.solver.forward(contracts.HOME)
        for dt in (0., -1., float('nan'), float('inf')):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                self.solver.step(goal, contracts.HOME, dt, np.zeros(7))
        for state in (np.zeros(6), np.full(7, np.nan), np.full(7, np.inf)):
            with self.assertRaises(ValueError):
                self.solver.solve(goal, state)
            with self.assertRaises(ValueError):
                self.solver.step(goal, contracts.HOME, 1/450, state)
        for bounds in (np.zeros((6, 2)), np.full((7, 2), np.nan)):
            with self.assertRaises(ValueError):
                Art7NativeIK(tools=self.tools, bounds_deg=bounds)
        crossed = np.asarray(BOUNDS_DEG).copy(); crossed[0] = [2., 1.]
        with self.assertRaises(ValueError):
            Art7NativeIK(tools=self.tools, bounds_deg=crossed)

    def test_period_bounds_only_tighten_the_original_period(self):
        q = contracts.HOME.copy(); dt=1/450
        goal = self.solver.forward(q).copy(); goal[0, 3] += 20.
        bounds = np.radians(np.asarray(BOUNDS_DEG))
        lower, upper = bounds[:, 0].copy(), bounds[:, 1].copy()
        lower[0] = upper[0] = q[0]
        for iterations in (1, 8, 40):
            result = self.solver.step(goal, q, dt, np.zeros(7),
                period_lower=lower, period_upper=upper, max_iterations=iterations)
            target = self.assert_valid_result(result, q, goal)
            self.assert_dynamic_bounds(target, q, np.zeros(7), dt)
            self.assertAlmostEqual(target[0], q[0], places=12)
        # The additional box conflicts with the physical period bounds; it
        # must not supersede the acceleration cap or manufacture a hold.
        lower[0] = q[0]+.01
        upper[0] = q[0]+.02
        result = self.solver.step(goal, q, dt, np.zeros(7),
            period_lower=lower, period_upper=upper, max_iterations=8)
        self.assertFalse(result['feasible'], result)
        self.assertIsNone(result['target'])

    def test_optional_retreat_keeps_original_guard_sign_and_raw_limit_rules(self):
        for sign6, sign7 in itertools.product((-1., 1.), repeat=2):
            q = np.radians([45.,75.,-100.,85.,135.,sign6*51.,sign7*37.8])
            inward_goal = q.copy()
            inward_goal[5:] -= np.radians([sign6*.1, sign7*.1])
            goal = self.solver.forward(inward_goal)
            normal = self.solver.step(goal, q, .05, np.zeros(7))
            self.assertFalse(normal['feasible'])
            bounds = np.radians(np.asarray(BOUNDS_DEG))
            lower, upper = bounds[:, 0].copy(), bounds[:, 1].copy()
            for i, sign in ((5, sign6), (6, sign7)):
                if sign > 0:lower[i], upper[i] = 0., q[i]
                else:lower[i], upper[i] = q[i], 0.
            with self.subTest(sign6=sign6, sign7=sign7):
                result = self.solver.step(goal, q, .05, np.zeros(7),
                    period_lower=lower, period_upper=upper, allow_guard_retreat=True,
                    max_iterations=20)
                target = self.assert_valid_result(result, q, goal)
                self.assert_dynamic_bounds(target, q, np.zeros(7), .05)
                self.assertTrue(np.all(abs(target[5:]) <= abs(q[5:])+1e-10))
                self.assertGreater(float(np.max(abs(target-q))), 1e-6)
                outward = np.zeros(7); outward[5:] = np.radians([sign6, sign7])
                denied = self.solver.step(goal, q, .05, np.zeros(7),
                    measured_velocity=outward, allow_guard_retreat=True)
                self.assertFalse(denied['feasible'], denied)
                self.assertIsNone(denied['target'])
        raw_outside = q.copy(); raw_outside[6] = math.radians(sign7*41.)
        result = self.solver.step(self.solver.forward(q), raw_outside,
            .05, np.zeros(7), allow_guard_retreat=True)
        self.assertFalse(result['feasible'], result)
        self.assertIsNone(result['target'])

    def test_reserve_override_cannot_reduce_measured_speed_protection(self):
        q=contracts.NEAR_WRIST.copy();goal=self.solver.forward(q)
        speed=np.zeros(7);speed[6]=math.radians(10.)
        # A future-speed reserve may reject a currently stationary point.
        result=self.solver.step(goal,q,1/450,np.zeros(7),reserve_velocity=speed)
        self.assertFalse(result['feasible'],result)
        self.assertIsNone(result['target'])
        # Conversely a zero prediction must never cancel a nonzero measurement.
        result=self.solver.step(goal,q,1/450,np.zeros(7),
            measured_velocity=speed,reserve_velocity=np.zeros(7))
        self.assertFalse(result['feasible'],result)
        self.assertIsNone(result['target'])


class NativeNumericalTests(unittest.TestCase):
    def test_native_box_qp_matches_reference_and_kkt_with_fixed_variables(self):
        rng=np.random.default_rng(20261101)
        for dimension in (1,2,3,7):
            for index in range(40):
                matrix=rng.normal(size=(dimension,dimension))
                h=matrix.T@matrix+.05*np.eye(dimension)
                g=rng.normal(size=dimension)*3.
                lower=rng.uniform(-1.,-.1,dimension)
                upper=rng.uniform(.1,1.,dimension)
                if index%3==0:lower[0]=upper[0]=.15
                if index%11==0:lower[:]=upper[:]
                native=solve_box_qp_native(h,g,lower,upper)
                reference=solve_box_qp(h,g,lower,upper)
                with self.subTest(dimension=dimension,index=index):
                    self.assertTrue(np.all(native>=lower-1e-12))
                    self.assertTrue(np.all(native<=upper+1e-12))
                    cost=lambda x: float(.5*x@h@x+g@x)
                    self.assertAlmostEqual(cost(native),cost(reference),places=8)
                    derivative=h@native+g
                    for i in range(dimension):
                        if lower[i]==upper[i]:continue
                        if native[i]<=lower[i]+1e-10:self.assertGreaterEqual(derivative[i],-1e-7)
                        elif native[i]>=upper[i]-1e-10:self.assertLessEqual(derivative[i],1e-7)
                        else:self.assertAlmostEqual(derivative[i],0.,places=7)

    def test_native_box_qp_rejects_invalid_hessian_or_bounds(self):
        for h,g,lower,upper in [(-np.eye(2),np.zeros(2),-np.ones(2),np.ones(2)),
            (np.array([[1.,2.],[0.,1.]]),np.zeros(2),-np.ones(2),np.ones(2)),
            (np.eye(2),np.zeros(2),np.ones(2),-np.ones(2)),
            (np.eye(2),np.array([np.nan,0.]),-np.ones(2),np.ones(2)),
            (np.empty((0,0)),np.zeros(0),np.zeros(0),np.zeros(0))]:
            with self.subTest(h=h),self.assertRaises(ValueError):
                solve_box_qp_native(h,g,lower,upper)

    def assert_pose_result(self, model, result, seed, goal):
        self.assertTrue(result['feasible'], result)
        self.assertTrue(result['success'], result)
        q=np.asarray(result['target'])
        WristGuard().arm_segment(seed.tolist(), q.tolist(), [0.]*7)
        limits=np.radians(np.asarray(BOUNDS_DEG))
        self.assertTrue(np.all(q>=limits[:,0]-1e-10))
        self.assertTrue(np.all(q<=limits[:,1]+1e-10))
        expected=independent_fk(q,model.tools,model.side)
        mm,deg=pose_error(goal,expected)
        self.assertLessEqual(mm,.020001);self.assertLessEqual(deg,.010001)
        mm,deg=pose_error(result['tcp'],expected)
        self.assertLess(mm,1e-7);self.assertLess(deg,1e-4)

    def test_interleaved_tool_and_arm_instances_do_not_share_configuration(self):
        models=[Art7NativeIK(tools=tool_config(mode),side=side)
                for mode,side in itertools.product(('flange','taccap'),('left','right'))]
        for index in range(16):
            model=models[index%len(models)]
            q=contracts.HOME.copy();q[index%7]+=math.radians(index*.1)
            actual=model.forward(q);expected=independent_fk(q,model.tools,model.side)
            mm,deg=pose_error(actual,expected)
            self.assertLess(mm,1e-8);self.assertLess(deg,1e-4)

    def test_random_fk_and_jacobian_against_independent_stdlib_model(self):
        rng=np.random.default_rng(20261030)
        for mode,side in itertools.product(('flange','taccap'),('left','right')):
            model=Art7NativeIK(tools=tool_config(mode),side=side)
            for _ in range(25):
                q=np.radians([rng.uniform(lo+.1,hi-.1) for lo,hi in BOUNDS_DEG])
                tcp=model.forward(q);expected=independent_fk(q,model.tools,side)
                mm,deg=pose_error(tcp,expected)
                self.assertLess(mm,1e-8);self.assertLess(deg,1e-4)
                jac=model.jacobian(q)
                for axis in range(7):
                    delta=np.eye(7)[axis]*1e-6
                    plus=independent_fk(q+delta,model.tools,side)
                    minus=independent_fk(q-delta,model.tools,side)
                    position=(plus[:3,3]-minus[:3,3])/2e-6
                    angular=rotation_log(plus[:3,:3]@minus[:3,:3].T)/2e-6
                    np.testing.assert_allclose(jac[:3,axis],position,atol=4e-7,rtol=1e-7)
                    np.testing.assert_allclose(jac[3:,axis],angular,atol=1e-8,rtol=1e-7)

    def test_static_small_targets_match_python_success_without_identical_joint_requirement(self):
        rng=np.random.default_rng(20261031)
        for mode,side in itertools.product(('flange','taccap'),('left','right')):
            config=tool_config(mode)
            model=Art7NativeIK(tools=config,side=side)
            original=Art7ConstrainedIK(tools=config,side=side)
            states=[contracts.HOME.copy(),contracts.NEAR_WRIST.copy(),
                    np.radians([45,75,-100,85,135,51,37.7])]
            for _ in range(6):
                sixth=rng.uniform(-60.,60.)
                seventh=rng.uniform(-.8,.8)*(cap(abs(sixth)+2.)-2.)
                states.append(np.radians([rng.uniform(-130,130),rng.uniform(-80,80),
                    rng.uniform(-130,130),rng.uniform(20,100),rng.uniform(-130,130),sixth,seventh]))
            for index,seed in enumerate(states):
                target=independent_fk(seed,config,side)
                target[index%3,3]+=(-1. if index%2 else 1.)*.5
                native=model.solve(target,seed,max_iterations=80)
                python=original.solve(target,seed,max_iterations=80)
                with self.subTest(mode=mode,side=side,pose=index):
                    self.assert_pose_result(model,native,seed,target)
                    self.assert_pose_result(original,python,seed,target)

    def test_joint_coupling_witness_uses_other_joints_and_preserves_complete_tcp(self):
        mirror=np.array([-1,1,-1,1,-1,1,-1])
        for mode,side in itertools.product(('flange','taccap'),('left','right')):
            model=Art7NativeIK(tools=tool_config(mode),side=side)
            seed=np.radians([45,75,-100,85,135,51,37.7])
            if side=='right':seed*=mirror
            changed=seed.copy();changed[6]+=math.radians(.1)*(1 if side=='left' else -1)
            goal=independent_fk(changed,model.tools,side)
            result=model.solve(goal,seed,max_iterations=80)
            self.assert_pose_result(model,result,seed,goal)
            self.assertGreater(float(np.abs(result['target'][:5]-seed[:5]).max()),math.radians(.01))
            self.assertLessEqual(abs(math.degrees(result['target'][6])),37.7+1e-7)


if __name__=='__main__':
    unittest.main(verbosity=2)
