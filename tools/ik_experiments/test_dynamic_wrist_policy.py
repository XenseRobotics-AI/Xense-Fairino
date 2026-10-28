"""Independent algebra, history and guard tests for predictive wrist bounds.

These tests use only the local native mathematics library and simulated state.
They deliberately avoid large trajectories and timing benchmarks.
"""
import math
from collections import deque
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'apps/fairino_tcp_probe'),
             str(ROOT/'apps/fairino_upper_probe')]

import numpy as np

from tools.ik_experiments.art7_native import Art7NativeIK
from tools.ik_experiments.dynamic_wrist_policy import PredictiveWristIK
from tcp_core import BOUNDS_DEG,multiply,pose_error,tcp_forward_matrix
from tool_frames import tool_config,for_arm
from wrist_guard import BOUNDARY,MARGIN_DEG,PREDICTION_SECONDS,WristGuard,cap


HOME=np.radians([45.,75.,-100.,85.,85.,0.,0.])
BOUNDARY_Q=np.radians([45.,75.,-100.,85.,135.,51.,37.7])
VMAX=math.radians(30.)
AMAX=math.radians(120.)


class PredictiveWristTests(unittest.TestCase):
    def setUp(self):
        self.backend=Art7NativeIK(tools=tool_config('flange'),side='left')
        self.policy=PredictiveWristIK(self.backend,feedback_age=.04)
        self.guard=WristGuard()

    def assert_command(self,result,reference,previous,dt,goal,measured=None,velocity=None):
        self.assertTrue(result['feasible'],result)
        q=np.asarray(result['target'])
        measured=reference if measured is None else measured
        velocity=np.zeros(7) if velocity is None else velocity
        qdot=(q-reference)/dt
        acceleration=(qdot-previous)/dt
        self.assertLessEqual(float(np.max(abs(qdot))),VMAX+1e-9)
        self.assertLessEqual(float(np.max(abs(acceleration))),AMAX+1e-7)
        self.guard.arm_segment(reference.tolist(),q.tolist(),velocity.tolist())
        self.guard.arm_segment(measured.tolist(),q.tolist(),velocity.tolist())
        fk=multiply(tcp_forward_matrix(q.tolist()),for_arm(self.backend.tools,'left'))
        mm,deg=pose_error(result['tcp'],fk)
        self.assertLess(mm,1e-7);self.assertLess(deg,1e-4)
        if result['success']:
            mm,deg=pose_error(goal,fk)
            self.assertLessEqual(mm,.020001);self.assertLessEqual(deg,.010001)
        return q,qdot

    def test_speed_cap_obeys_braking_and_feedback_horizon_algebra(self):
        rng=np.random.default_rng(20261102)
        accepted=0
        for _ in range(150):
            sixth=rng.uniform(-50.,50.)
            seventh=rng.uniform(-.6,.6)*(cap(abs(sixth)+2.)-2.)
            q=np.radians([45,75,-100,85,135,sixth,seventh])
            measured=q.copy();measured[5:]+=np.radians(rng.uniform(-.01,.01,2))
            velocity=np.zeros(7);velocity[5:]=np.radians(rng.uniform(-.5,.5,2))
            previous=np.zeros(7);dt=float(rng.choice([1/450,1/90,.01]))
            amax=np.full(7,AMAX);vmax=np.full(7,VMAX)
            braking=float(rng.uniform(.1,3.))
            policy=PredictiveWristIK(self.backend,feedback_age=float(rng.uniform(0,.08)),
                                      braking_acceleration=braking)
            plan,_=policy._bounds(q,measured,velocity,previous,dt,vmax,amax)
            if plan is None:continue
            accepted+=1
            _,index,lower,upper,caps,speed,old_speed,position=plan
            expected=np.radians(np.asarray(BOUNDARY[index])-MARGIN_DEG)
            np.testing.assert_allclose(caps,expected,atol=1e-14,rtol=0.)
            gap=caps-position
            effective_braking=min(braking,AMAX)
            used=(PREDICTION_SECONDS*np.maximum(old_speed,speed)
                  +(policy.feedback_age+dt)*speed+speed*speed/(2*effective_braking))
            self.assertTrue(np.all(used<=gap+1e-11),(used,gap))
            self.assertTrue(np.all(PREDICTION_SECONDS*old_speed<=gap+1e-11))
            self.assertTrue(np.all(speed<=vmax[-2:]+1e-12))
            # Every endpoint of the returned rectangular period interval must
            # obey the new wrist speed cap, not just the optimizer's output.
            self.assertTrue(np.all(abs((lower[-2:]-q[-2:])/dt)<=speed+1e-9))
            self.assertTrue(np.all(abs((upper[-2:]-q[-2:])/dt)<=speed+1e-9))
        self.assertGreater(accepted,130)

    def test_braking_assumption_cannot_exceed_command_acceleration_limit(self):
        q=HOME.copy();q[5:]=np.radians([51.,37.2])
        velocity=np.zeros(7);velocity[6]=math.radians(1.)
        args=(q,q,velocity,np.zeros(7),1/450,np.full(7,VMAX),np.full(7,AMAX))
        standard=PredictiveWristIK(self.backend,feedback_age=.04,
                                     braking_acceleration=AMAX)._bounds(*args)[0]
        optimistic=PredictiveWristIK(self.backend,feedback_age=.04,
                                       braking_acceleration=1000.)._bounds(*args)[0]
        self.assertIsNotNone(standard);self.assertIsNotNone(optimistic)
        self.assertEqual(standard[1],optimistic[1])
        np.testing.assert_allclose(standard[5],optimistic[5],atol=1e-14,rtol=0.)
        gap=standard[4]-standard[7]
        used=(PREDICTION_SECONDS*np.maximum(standard[6],standard[5])
              +(.04+1/450)*standard[5]+standard[5]**2/(2*AMAX))
        self.assertTrue(np.all(used<=gap+1e-11))

    def test_zero_margin_locks_last_axis_but_allows_other_joints_to_progress(self):
        q=BOUNDARY_Q.copy();changed=q.copy();changed[6]+=math.radians(.1)
        goal=self.backend.forward(changed);dt=1/450;previous=np.zeros(7)
        result=self.policy.step(goal,q,dt,previous,max_iterations=4)
        target,qdot=self.assert_command(result,q,previous,dt,goal)
        self.assertLess(abs(target[6]-q[6]),1e-12)
        self.assertGreater(float(np.max(abs(target[:5]-q[:5]))),1e-8)
        self.assertLessEqual(result['wrist_speed_cap_deg_s'][1],1e-7)
        before=pose_error(goal,self.backend.forward(q))
        after=pose_error(goal,self.backend.forward(target))
        self.assertLess(after[0]**2+(math.radians(after[1])*200)**2,
                        before[0]**2+(math.radians(before[1])*200)**2)

    def test_old_speed_and_position_are_not_cleared_by_a_new_goal(self):
        q=HOME.copy();goal=self.backend.forward(q);goal[0,3]+=20.
        dt=1/450;previous=np.zeros(7)
        first=self.policy.step(goal,q,dt,previous,max_iterations=4)
        target,sent_speed=self.assert_command(first,q,previous,dt,goal)
        self.assertGreater(float(np.max(abs(sent_speed[-2:]))),1e-5)
        # Accept the prior command, then request an unrelated target. Actual
        # feedback reports zero velocity, but the previous command remains a
        # required witness for the bound until the feedback-age horizon passes.
        other=self.backend.forward(target);other[1,3]-=10.
        second=self.policy.step(other,target,dt,sent_speed,
            measured=target,measured_velocity=np.zeros(7),max_iterations=4)
        self.assert_command(second,target,sent_speed,dt,other)
        speed_bound=np.radians(second['current_speed_bound_deg_s'])
        position_bound=np.radians(second['historical_position_bound_deg'])
        self.assertTrue(np.all(speed_bound>=abs(sent_speed[-2:])-1e-12))
        self.assertTrue(np.all(position_bound>=abs(target[-2:])-1e-12))
        # Independently inject an older, still relevant command to ensure both
        # maxima survive even if current q/measurement/previous velocity shrink.
        historical_q=target.copy();historical_q[5:]=np.radians([5.,7.])
        historical_speed=np.zeros(7);historical_speed[5:]=np.radians([2.,3.])
        self.policy._history.append((self.policy._clock,historical_q,historical_speed))
        plan,_=self.policy._bounds(HOME.copy(),HOME.copy(),np.zeros(7),np.zeros(7),
                                  dt,np.full(7,VMAX),np.full(7,AMAX))
        self.assertIsNotNone(plan)
        self.assertTrue(np.all(plan[6]>=abs(historical_speed[-2:])-1e-12))
        self.assertTrue(np.all(plan[7]>=abs(historical_q[-2:])-1e-12))

    def test_retries_do_not_commit_or_age_history_until_reference_acknowledges_output(self):
        q=HOME.copy();previous=np.zeros(7);dt=1/450
        goal=self.backend.forward(q);goal[0,3]+=20.
        first=self.policy.step(goal,q,dt,previous,max_iterations=4)
        self.assert_command(first,q,previous,dt,goal)
        self.assertEqual(self.policy._clock,0.)
        self.assertEqual(len(self.policy._history),0)
        for _ in range(25):
            retry=self.policy.step(goal,q,dt,previous,max_iterations=4)
            self.assert_command(retry,q,previous,dt,goal)
        self.assertEqual(self.policy._clock,0.)
        self.assertEqual(len(self.policy._history),0)
        accepted=np.asarray(retry['target']);accepted_speed=(accepted-q)/dt
        second=self.policy.step(goal,accepted,dt,accepted_speed,max_iterations=4)
        self.assert_command(second,accepted,accepted_speed,dt,goal)
        self.assertAlmostEqual(self.policy._clock,dt,places=14)
        self.assertEqual(len(self.policy._history),1)
        # The second candidate is rejected by the caller. Reuse the last
        # accepted reference and velocity: neither the clock nor sent history
        # may advance just because another candidate was computed.
        goal2=goal.copy();goal2[1,3]+=1.
        self.policy.step(goal2,accepted,dt,accepted_speed,max_iterations=4)
        self.assertAlmostEqual(self.policy._clock,dt,places=14)
        self.assertEqual(len(self.policy._history),1)

    def test_reference_discontinuity_requires_explicit_reset(self):
        q=HOME.copy();goal=self.backend.forward(q);goal[0,3]+=20.
        self.policy.step(goal,q,1/450,np.zeros(7))
        moved=q.copy();moved[0]+=.01
        with self.assertRaises(ValueError):
            self.policy.step(self.backend.forward(moved),moved,1/450,np.zeros(7))
        self.policy.reset()
        result=self.policy.step(self.backend.forward(moved),moved,1/450,np.zeros(7))
        self.assertTrue(result['feasible'],result)
        self.assertEqual(self.policy._clock,0.)

    def test_explicit_commit_handles_hold_frames_and_rejects_old_or_reset_tokens(self):
        q=HOME.copy();goal=self.backend.forward(q);dt=1/450
        first=self.policy.step(goal,q,dt,np.zeros(7))
        second=self.policy.step(goal,q,dt,np.zeros(7))
        with self.assertRaises(ValueError):self.policy.commit(first)
        self.assertEqual(self.policy._clock,0.)
        self.policy.commit(second)
        self.assertAlmostEqual(self.policy._clock,dt,places=14)
        with self.assertRaises(ValueError):self.policy.commit(second)
        for _ in range(5):
            current=self.policy.step(goal,q,dt,np.zeros(7))
            self.policy.commit(current)
        self.assertAlmostEqual(self.policy._clock,6*dt,places=14)
        pending=self.policy.step(goal,q,dt,np.zeros(7))
        self.policy.reset()
        replacement=self.policy.step(goal,q,dt,np.zeros(7))
        self.assertNotEqual(pending['proposal_id'],replacement['proposal_id'])
        with self.assertRaises(ValueError):self.policy.commit(pending)
        self.assertEqual(self.policy._clock,0.)
        self.policy.commit(replacement)
        self.assertAlmostEqual(self.policy._clock,dt,places=14)

    def test_illegal_feedback_age_and_braking_parameters_are_rejected(self):
        for value in (-.1,float('nan'),float('inf')):
            with self.subTest(age=value),self.assertRaises(ValueError):
                PredictiveWristIK(self.backend,feedback_age=value)
        for value in (0.,-1.,float('nan'),float('inf'),np.ones(6)):
            with self.subTest(braking=value),self.assertRaises(ValueError):
                policy=PredictiveWristIK(self.backend,braking_acceleration=value)
                policy.step(self.backend.forward(HOME),HOME,1/450,np.zeros(7))
        for dt in (0.,-1.,float('nan'),float('inf')):
            with self.subTest(dt=dt),self.assertRaises(ValueError):
                self.policy.step(self.backend.forward(HOME),HOME,dt,np.zeros(7))

    def test_invalid_current_or_delayed_state_cannot_be_reported_as_safe_hold(self):
        reference=BOUNDARY_Q.copy();measured=reference.copy()
        measured[6]+=math.radians(.1)
        result=self.policy.step(self.backend.forward(reference),reference,1/450,np.zeros(7),
                                measured=measured,measured_velocity=np.zeros(7))
        self.assertFalse(result['feasible'],result);self.assertIsNone(result['target'])
        velocity=np.zeros(7);velocity[6]=math.radians(2.)
        result=self.policy.step(self.backend.forward(reference),reference,1/450,np.zeros(7),
                                measured_velocity=velocity)
        self.assertFalse(result['feasible'],result);self.assertIsNone(result['target'])

    def test_guard_implementation_and_boundaries_remain_unchanged(self):
        before=tuple(BOUNDARY)
        methods=(WristGuard.arm_segment,WristGuard.clearance,WristGuard._pair_motion)
        q=HOME.copy();goal=self.backend.forward(q);goal[0,3]+=.1
        result=self.policy.step(goal,q,1/450,np.zeros(7))
        self.assert_command(result,q,np.zeros(7),1/450,goal)
        self.assertEqual(tuple(BOUNDARY),before)
        self.assertEqual(methods,(WristGuard.arm_segment,WristGuard.clearance,WristGuard._pair_motion))
        self.assertTrue(self.backend.guard.enabled)

    def test_brake_monotonically_reaches_zero_within_the_discrete_acceleration_bound(self):
        near=BOUNDARY_Q.copy();near[6]-=math.radians(.5)
        fixtures=[(HOME,np.radians([6.,-4.,5.,-3.,2.,8.,-7.])),
                  (HOME,np.radians([-6.,4.,-5.,3.,-2.,-8.,7.])),
                  (near,np.radians([4.,-3.,2.,-1.,2.,.8,.7])),
                  (near,np.radians([-4.,3.,-2.,1.,-2.,-.8,-.7]))]
        for initial,initial_velocity in fixtures:
            for dt in (1/450,1/90):
                for tau,delay in ((0.,0.),(.03,.02)):
                    with self.subTest(wrist=np.degrees(initial[5:]),
                                      velocity=np.degrees(initial_velocity),dt=dt,tau=tau):
                        policy=PredictiveWristIK(self.backend,feedback_age=.04)
                        q=initial.copy();previous=initial_velocity.copy()
                        # Start from a consistent finite tracking lag, rather
                        # than claiming nonzero motor velocity at zero position
                        # error in a first-order follower.
                        actual=q-tau*previous
                        actual_velocity=previous.copy()
                        history=deque([(0.,actual.copy(),actual_velocity.copy())])
                        feedback_q=actual.copy();feedback_v=actual_velocity.copy()
                        last_slot=-1;t=0.
                        maximum_steps=int(math.ceil(float(np.max(abs(previous)))/(AMAX*dt)))
                        stopped=False;stopping_point=None
                        for index in range(maximum_steps+4):
                            t+=dt;slot=math.floor((t+1e-12)*100.)
                            if slot>last_slot:
                                while len(history)>1 and history[1][0]<=t-delay:
                                    history.popleft()
                                feedback_q=history[0][1].copy();feedback_v=history[0][2].copy()
                                last_slot=slot
                            result=policy.brake(q,dt,previous,measured=feedback_q,
                                                measured_velocity=feedback_v)
                            self.assertTrue(result['feasible'],result)
                            target=np.asarray(result['target'])
                            # The expected endpoint is computed independently;
                            # it is joint deceleration, not IK towards a frozen TCP.
                            expected_velocity=np.sign(previous)*np.maximum(abs(previous)-AMAX*dt,0.)
                            expected_target=q+dt*expected_velocity
                            np.testing.assert_allclose(target,expected_target,atol=2e-12,rtol=0.)
                            speed=(target-q)/dt
                            self.assertTrue(np.all(abs(speed)<=abs(previous)+1e-10))
                            self.assertTrue(np.all(speed*previous>=-1e-12))
                            self.assertLessEqual(float(np.max(abs(speed-previous)/dt)),AMAX+1e-7)
                            self.guard.arm_segment(q.tolist(),target.tolist(),actual_velocity.tolist())
                            self.guard.arm_segment(actual.tolist(),target.tolist(),actual_velocity.tolist())
                            self.guard.arm_segment(q.tolist(),target.tolist(),feedback_v.tolist())
                            self.guard.arm_segment(feedback_q.tolist(),target.tolist(),feedback_v.tolist())
                            if index+1>=maximum_steps:
                                self.assertLess(float(np.max(abs(speed))),1e-10)
                                self.assertTrue(result['stopped'],result)
                            if result['stopped']:
                                if stopping_point is None:stopping_point=target.copy()
                                else:np.testing.assert_allclose(target,stopping_point,atol=1e-12,rtol=0.)
                                stopped=True
                            policy.commit(result)
                            before=actual.copy()
                            actual=target.copy() if tau==0. else actual+(1-math.exp(-dt/tau))*(target-actual)
                            actual_velocity=(actual-before)/dt
                            history.append((t,actual.copy(),actual_velocity.copy()))
                            q=target;previous=speed
                        self.assertTrue(stopped)

    def test_brake_rejects_states_with_no_legal_stopping_step(self):
        q=HOME.copy();q[0]=math.radians(174.999)
        previous=np.zeros(7);previous[0]=.3
        result=self.policy.brake(q,.01,previous)
        self.assertFalse(result['feasible'],result)
        self.assertFalse(result['stopped'],result)
        self.assertIsNone(result['target'])
        self.policy.reset()
        velocity=np.zeros(7);velocity[6]=math.radians(2.)
        result=self.policy.brake(BOUNDARY_Q.copy(),1/450,np.zeros(7),measured_velocity=velocity)
        self.assertFalse(result['feasible'],result)
        self.assertFalse(result['stopped'],result)
        self.assertIsNone(result['target'])
        for dt in (0.,-1.,float('nan'),float('inf')):
            with self.subTest(dt=dt),self.assertRaises(ValueError):
                self.policy.brake(HOME.copy(),dt,np.zeros(7))
        for previous in (np.zeros(6),np.full(7,np.nan),np.full(7,np.inf)):
            with self.assertRaises(ValueError):
                self.policy.brake(HOME.copy(),1/450,previous)


if __name__=='__main__':
    unittest.main(verbosity=2)
