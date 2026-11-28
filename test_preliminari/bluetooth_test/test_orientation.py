"""Test senza scheda; eseguire: python -m unittest -v test_orientation.py"""
import asyncio
import math
import re
import unittest
from unittest.mock import AsyncMock, patch

import numpy as np
from ahrs.filters import Madgwick
from dbus_fast import Message, MessageType, Variant

from orientation import (Orientation, conjugate, euler, from_euler, initial_attitude,
                         madgwick, matrix, multiply, rev2_axes, unit)
from viewer import create_app
import test_receiver as receiver


class MathTests(unittest.TestCase):
    def assertQuat(self, a, b, tolerance=1e-7):
        self.assertLess(1-abs(np.dot(unit(a), unit(b))), tolerance)

    def test_known_euler_and_wrap(self):
        for angles in [(30,0,0),(0,40,0),(0,0,90),(0,0,179),(0,0,-179),(20,-35,170)]:
            q = from_euler(*np.radians(angles))
            np.testing.assert_allclose(euler(q), angles, atol=1e-9)
        q1, q2 = (from_euler(0,0,np.radians(x)) for x in (179,-179))
        self.assertAlmostEqual(euler(multiply(conjugate(q1),q2))[2],2)

    def test_relative_is_not_euler_subtraction(self):
        ref=from_euler(.4,.3,1.2); delta=from_euler(.5,-.2,.8)
        q=multiply(ref,delta)
        self.assertQuat(multiply(conjugate(ref),q),delta)

    def test_initialization_tilted(self):
        for angles in [(0,0,0),(.4,-.3,2.9),(-.7,.5,-2.9)]:
            q=from_euler(*angles); R=matrix(q)
            self.assertQuat(initial_attitude(R.T@[0,0,9.80665],R.T@[25,0,40]),q)

    def test_sensor_mapping(self):
        a,g,m=rev2_axes([1,2,3],[180,90,-180],[4,5,6])
        np.testing.assert_allclose(a,[1,2,3]); np.testing.assert_allclose(g,[math.pi,math.pi/2,-math.pi])
        np.testing.assert_allclose(m,[4,-5,6])

    def test_against_ahrs_reference(self):
        rng=np.random.default_rng(42); reference=Madgwick(gain=.06)
        for _ in range(100):
            q=unit(rng.normal(size=4)); a=rng.normal(size=3); g=rng.normal(size=3); m=rng.normal(size=3)
            np.testing.assert_allclose(madgwick(q,g,a,m,.012),reference.updateMARG(q,g,a,m,dt=.012),atol=1e-12)
            np.testing.assert_allclose(madgwick(q,g,a,None,.012),reference.updateIMU(q,g,a,dt=.012),atol=1e-12)

    def test_zero_gyro_converges_and_zero_gradient_safe(self):
        q=from_euler(.2,-.1,.3)
        for _ in range(2000):
            q=madgwick(q,[0,0,0],[0,0,9.80665],[25,0,40],.01)
        self.assertLess(np.linalg.norm(euler(q)),.2)
        np.testing.assert_allclose(madgwick([1,0,0,0],[0,0,0],[0,0,1],[1,0,0],.01),[1,0,0,0])

    def test_synthetic_rotation_all_axes(self):
        for axis in range(3):
            q=np.array([1.,0,0,0]); rate=np.zeros(3);rate[axis]=.4
            for i in range(1,501):
                angles=np.zeros(3);angles[axis]=.4*i*.01
                truth=from_euler(*angles);R=matrix(truth)
                q=madgwick(q,rate,R.T@[0,0,9.80665],R.T@[24,0,39],.01)
            self.assertQuat(q,truth,1e-4)

    def test_frontend_basis(self):
        B=matrix(from_euler(-math.pi/2,0,0))
        np.testing.assert_allclose(B@[1,2,3],[1,3,-2],atol=1e-12)
        for angles in [(0,0,math.pi),(.5,0,0),(0,.5,0)]:
            q=from_euler(*angles); b=from_euler(-math.pi/2,0,0)
            np.testing.assert_allclose(matrix(multiply(multiply(b,q),conjugate(b))),B@matrix(q)@B.T,atol=1e-12)


class StateTests(unittest.TestCase):
    def setUp(self):
        self.t=0.;self.state=Orientation(clock=lambda:self.t)

    def feed(self, **kwargs):
        self.t+=.02
        m=kwargs.get('mag',[24+1e-3*math.sin(self.t),0,39])
        self.state.feed([5,0,0,9.80665,0,0,0,*m],self.t)

    def initialize(self):
        for _ in range(90):self.feed()

    def test_wait_stable_reset_stale_and_recovery(self):
        self.assertEqual(self.state.snapshot()['status'],'attesa')
        self.assertFalse(self.state.reset());self.feed()
        self.assertEqual(self.state.snapshot()['status'],'inizializzazione')
        self.initialize();self.assertTrue(self.state.snapshot()['valid'])
        self.state.q=from_euler(.3,.4,.5)
        self.assertTrue(self.state.reset())
        np.testing.assert_allclose(self.state.snapshot()['rpy'],[0,0,0],atol=1e-10)
        self.t+=.6;self.assertEqual(self.state.snapshot()['status'],'stale')
        self.assertIsNone(self.state.snapshot()['quaternion']);self.assertFalse(self.state.reset())
        self.feed();self.assertEqual(self.state.snapshot()['status'],'inizializzazione')
        self.initialize();self.assertTrue(self.state.snapshot()['valid'])
        self.state.connection(False);self.assertEqual(self.state.snapshot()['status'],'disconnesso')
        self.assertIsNone(self.state.snapshot()['rpy'])

    def test_mag_zero_frozen_and_return(self):
        self.initialize()
        self.assertEqual(self.state.snapshot()['mode'],'MARG')
        self.feed(mag=[0,0,0]);self.assertEqual(self.state.snapshot()['mode'],'IMU')
        for _ in range(30):self.feed(mag=[24,0,39])
        self.assertEqual(self.state.snapshot()['mode'],'IMU')
        self.feed(mag=[24.1,0,39]);self.assertEqual(self.state.snapshot()['mode'],'MARG')

    def test_motion_prevents_initialization(self):
        for i in range(150):
            self.t=i*.02;self.state.feed([0,0,0,9.80665,20,0,0,24,0,39],self.t)
        self.assertFalse(self.state.snapshot()['valid'])

    def test_invalid_and_buttons(self):
        self.state.feed([0]*10,0);self.state.feed([0]+[float('nan')]*9,0)
        self.assertEqual(self.state.invalid,2);self.assertFalse(self.state.snapshot()['valid'])
        self.initialize();self.assertEqual(self.state.snapshot()['buttons'],[True,False,True,False,False,False,False])

    def test_flask_routes_reset_security(self):
        app=create_app(self.state);client=app.test_client()
        html=client.get('/').text; token=re.search('name="reset-token" content="([^"]+)"',html)[1]
        self.assertFalse(client.get('/api/state').json['valid'])
        self.assertEqual(client.post('/api/reset').status_code,403)
        self.assertEqual(client.get('/api/reset').status_code,405)
        headers={'X-Reset-Token':token}
        self.assertEqual(client.post('/api/reset',headers=headers).status_code,409)
        self.initialize();self.assertEqual(client.post('/api/reset',headers=headers).status_code,200)
        self.assertEqual(client.get('/api/state').json['resets'],1)
        with client.get('/static/app.js') as response:
            self.assertEqual(response.status_code,200)


class BleTests(unittest.IsolatedAsyncioTestCase):
    async def test_session_cancellation_removes_matches_without_disconnect(self):
        state=Orientation();stats=receiver.FrameStats();calls=[]
        class Bus:
            def add_message_handler(self,h):self.h=h
            def remove_message_handler(self,h):self.h=None
        bus=Bus()
        async def fake_call(_bus,**kw):
            calls.append(kw['member'])
            return Message(message_type=MessageType.METHOD_RETURN,reply_serial=1)
        props={'Alias':Variant('s','IMU_BTN'),'Address':Variant('s','18:FE:03:D9:D4:4B')}
        with patch.object(receiver,'wait_for_device',AsyncMock(return_value=('/dev',props,'/dev/char',{}))), patch.object(receiver,'call',fake_call):
            task=asyncio.create_task(receiver.run_session(bus,stats,None,state))
            await asyncio.sleep(.01)
            self.assertTrue(state.connected)
            task.cancel()
            with self.assertRaises(asyncio.CancelledError):await task
        self.assertEqual(calls,['AddMatch','AddMatch','StartNotify','StopNotify','RemoveMatch','RemoveMatch'])
        self.assertFalse(state.connected);self.assertIsNone(bus.h)

    def test_format_preserved(self):
        line=receiver.format_frame(123.4,(5,*range(9)))
        self.assertIn('[Hz: 123.4] BTN: 1010000',line)
        self.assertEqual(receiver.PAYLOAD_SIZE,37)


if __name__=='__main__':unittest.main()
