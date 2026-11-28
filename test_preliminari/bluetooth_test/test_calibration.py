"""Verifiche sintetiche della calibrazione; nessun accesso a hardware reale."""
import copy
import json
from pathlib import Path
import re
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from calibration import Calibration, ProfileStore, fit_magnetometer
from orientation import Orientation, madgwick
from viewer import create_app

DEVICE = '18:FE:03:D9:D4:4B'
S = np.array([[1.2, .2, .1], [.2, .9, -.1], [.1, -.1, 1.1]])
OFFSET = np.array([12., -8., 5.])
BIAS = np.radians([2.1, -1.5, .7])


def cloud(n=500, seed=42):
    rng = np.random.default_rng(seed)
    u = rng.normal(size=(n, 3))
    u /= np.linalg.norm(u, axis=1)[:, None]
    return 45*u, rng


def observations(n=500):
    v, rng = cloud(n)
    return v@S.T+OFFSET+rng.normal(0, .08, v.shape)


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.points = observations()
        cls.profile = fit_magnetometer(cls.points)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = ProfileStore(self.temp.name)
        self.c = Calibration(self.store, device=DEVICE)
        self.t = 0.
        self.rng = np.random.default_rng(19)

    def frame(self, a=None, g=None, m=None, dt=.025):
        self.t += dt
        a = np.array([0, 0, 9.80665]) + self.rng.normal(0, .005, 3) if a is None else np.asarray(a)
        g = BIAS+self.rng.normal(0, np.radians(.02), 3) if g is None else np.asarray(g)
        m = [25, 10, 40] if m is None else m
        self.c.feed(a, g, np.array(m), self.t)

    def gyro(self):
        for _ in range(175):
            self.frame()
        self.assertIsNotNone(self.c.bias)

    def test_gyro_nonzero_bias_and_stationarity(self):
        self.gyro()
        np.testing.assert_allclose(self.c.bias, BIAS, atol=np.radians(.01))
        self.assertEqual(self.c.phase, 'mag_pronto')
        self.assertFalse(self.c.allowed)

    def test_gyro_motion_variance_resets_without_bias_threshold(self):
        for _ in range(60):
            self.frame()
        self.assertGreater(self.c.elapsed, 1)
        self.frame(g=BIAS+np.radians([2, 0, 0]))
        self.assertEqual(self.c.elapsed, 0)
        self.assertIn('Movimento', self.c.message)
        self.assertIsNone(self.c.bias)
        for i in range(400):
            self.frame(g=BIAS+np.radians([(-1)**i, 0, 0]))
        self.assertIsNone(self.c.bias)
        for i in range(400):
            self.frame(a=[(-1)**i, 0, 9.80665])
        self.assertIsNone(self.c.bias)

    def test_gyro_absent_stale_duplicate_and_timeout(self):
        self.assertEqual(self.c.snapshot(100, False)['progress'], 0)
        for _ in range(40):
            self.frame()
        self.c.snapshot(self.t+1, True)
        self.assertEqual(self.c.elapsed, 0)
        for _ in range(220):
            self.frame(a=[0, 0, 9.80665], g=BIAS)
        self.assertIsNone(self.c.bias)
        self.assertEqual(self.c.elapsed, 0)
        self.frame(dt=181)
        self.assertEqual(self.c.phase, 'errore')

    def test_spd_offdiagonal_holdout_recovery(self):
        profile = self.profile
        np.testing.assert_allclose(profile['offset'], OFFSET, atol=.1)
        expected = np.linalg.inv(S)*np.linalg.det(S)**(1/3)
        np.testing.assert_allclose(profile['matrix'], expected, atol=.003)
        v, _ = cloud(600, seed=100)
        recovered = (v@S.T+OFFSET-profile['offset'])@np.array(profile['matrix']).T
        error = np.linalg.norm(recovered/np.linalg.det(S)**(1/3)-v, axis=1)
        self.assertLess(float(np.max(error)), .2)
        self.assertLess(np.linalg.norm(np.array(profile['matrix'])-np.diag(np.diag(profile['matrix']))), 1)
        self.assertGreater(abs(profile['matrix'][0][1]), .1)

    def test_moderate_outliers_are_trimmed(self):
        p = self.points.copy()
        p[:35] += self.rng.normal(0, 50, (35, 3))
        result = fit_magnetometer(p)
        np.testing.assert_allclose(result['offset'], OFFSET, atol=.2)
        self.assertLess(result['quality']['inlier_fraction'], .97)

    def test_reject_planar_degenerate_insufficient_and_bad_fit(self):
        angle = np.linspace(0, 2*np.pi, 300)
        planar = np.column_stack([40*np.cos(angle), 40*np.sin(angle), np.zeros(300)])
        for p in [planar, np.ones((200,3)), self.points[:100], self.rng.uniform(-80,80,(400,3)),
                  self.points*np.array([1,1,.001])]:
            with self.subTest(shape=p.shape), self.assertRaises(ValueError):
                fit_magnetometer(p)

    def test_collection_dedup_rate_limit_failure_retry_cancel(self):
        self.gyro()
        self.c.action('mag', self.t, True)
        for _ in range(1000):
            self.frame(m=self.points[0], dt=.005)
        self.assertEqual(len(self.c.samples), 1)
        for point in self.points[:20]:
            self.frame(m=point, dt=.01)
        self.assertLessEqual(len(self.c.samples), 3)
        self.c.action('verifica', self.t, True)
        self.assertEqual(self.c.phase, 'errore')
        self.assertIsNone(self.c.profile)
        self.c.action('riprova', self.t, True)
        self.assertEqual(self.c.phase, 'mag')
        self.assertEqual(len(self.c.samples), 0)
        self.c.action('annulla', self.t, True)
        self.assertEqual(self.c.phase, 'annullato')

    def test_success_persistence_and_cancel_preserves_previous(self):
        self.gyro()
        self.c.action('mag', self.t, True)
        for point in self.points:
            self.frame(m=point, dt=.1)
        self.c.action('verifica', self.t, True)
        self.assertEqual(self.c.phase, 'riferimento')
        self.assertTrue(self.c.allowed)
        saved = copy.deepcopy(self.c.profile)
        reloaded = Calibration(self.store, device=DEVICE)
        self.assertEqual(reloaded.profile, saved)
        self.assertIsNone(reloaded.bias)
        self.assertEqual(reloaded.phase, 'gyro')
        self.c.action('mag', self.t, True)
        for i in range(220):
            self.frame(m=[40*np.cos(i/10),40*np.sin(i/10),1], dt=.1)
        self.c.action('verifica', self.t, True)
        self.assertEqual(self.c.phase, 'errore')
        self.assertEqual(self.c.profile, saved)
        self.c.action('annulla', self.t, True)
        self.assertEqual(self.c.profile, saved)
        self.assertEqual(self.c.phase, 'riferimento')

    def test_profile_isolation_and_corruption(self):
        saved = self.store.save(self.profile, DEVICE, False)
        self.assertIsNone(self.store.load(DEVICE, True))
        self.assertIsNone(self.store.load('11:22:33:44:55:66', False))
        self.store.save(self.profile, 'SIMULATED', True)
        with self.assertRaises(ValueError):
            self.store.load('SIMULATED', False)
        for corrupt in [[], {**saved,'schema':999}, {**saved,'device':'11:22:33:44:55:66'},
                        {**saved,'demo':True}, {**saved,'matrix':[[1,0,0],[0,-1,0],[0,0,1]]},
                        {**saved,'matrix':[[1,1,0],[0,1,0],[0,0,1]]}, {**saved,'offset':[float('nan'),0,0]}]:
            self.store.path(DEVICE, False).write_text(json.dumps(corrupt))
            c = Calibration(self.store, device=DEVICE)
            self.assertIsNone(c.profile)
            self.assertIn('ignorato', c.notice)
        self.store.save(self.profile, DEVICE, False)
        self.c.bind('11:22:33:44:55:66')
        self.assertIsNone(self.c.profile)
        self.assertIsNone(self.c.bias)

    def test_stale_mag_discards_collection_and_cannot_verify(self):
        self.gyro()
        self.c.action('mag', self.t, True)
        self.frame(m=self.points[0])
        self.c.snapshot(self.t+1, True)
        self.assertEqual(self.c.phase, 'errore')
        self.assertEqual(len(self.c.samples), 0)
        with self.assertRaises(ValueError):
            self.c.action('verifica', self.t+1, True)

    def test_pipeline_correction_once_filter_unchanged_normalized(self):
        self.c.profile = self.store.save(self.profile, DEVICE, False)
        self.gyro()
        state = Orientation(clock=lambda:self.t, calibration=self.c)
        def feed(g):
            self.t += .025
            state.feed([0,0,0,9.80665,*np.degrees(g),25,-10,40], self.t)
        for _ in range(65):
            feed(BIAS)
        self.assertTrue(state.snapshot()['valid'])
        with patch('orientation.madgwick', wraps=madgwick) as spy:
            feed(BIAS+np.radians([5,0,0]))
            np.testing.assert_allclose(spy.call_args.args[1], np.radians([5,0,0]), atol=np.radians(.01))
            expected=np.array(self.c.profile['matrix'])@(np.array([25,10,40])-self.c.profile['offset'])
            # Mag is held constant: force a new magnetic sample to avoid fallback.
            self.t += .025
            state.feed([0,0,0,9.80665,*np.degrees(BIAS),26,-10,40],self.t)
            expected=np.array(self.c.profile['matrix'])@(np.array([26,10,40])-self.c.profile['offset'])
            np.testing.assert_allclose(spy.call_args.args[3],expected)
        self.assertAlmostEqual(np.linalg.norm(state.q),1)
        original=self.c.corrected(BIAS,np.array([26,10,40]))[1]
        self.c.profile=self.store.load(DEVICE,False)
        np.testing.assert_allclose(self.c.corrected(BIAS,np.array([26,10,40]))[1],original)
        state.calibration_action('mag')
        self.assertIsNone(state.q)
        self.assertFalse(state.reset())
        state.calibration_action('annulla')
        self.assertIsNone(state.q)

    def test_save_failure_keeps_previous_and_rebind_clears_freshness(self):
        self.c.profile=self.store.save(self.profile,DEVICE,False)
        self.gyro()
        self.assertEqual(self.c.phase,'riferimento')  # Profilo presente: non impone mag.
        old=copy.deepcopy(self.c.profile)
        self.c.action('mag',self.t,True)
        for point in self.points:
            self.frame(m=point,dt=.1)
        with patch.object(self.store,'save',side_effect=OSError('disco non scrivibile')):
            self.c.action('verifica',self.t,True)
        self.assertEqual(self.c.phase,'errore')
        self.assertEqual(self.c.profile,old)
        self.assertEqual(self.store.load(DEVICE,False),old)
        self.c.action('gyro',self.t,True)
        self.c.phase='errore'
        with self.assertRaises(ValueError):
            self.c.action('verifica',self.t,True)
        self.c.bind('11:22:33:44:55:66')
        self.assertIsNone(self.c.last)
        self.assertEqual(self.c.samples,[])
        self.assertIsNone(self.c.profile)

    def test_api_actions_auth_state_and_reset_distinction(self):
        state=Orientation(clock=lambda:self.t,calibration=self.c)
        client=create_app(state).test_client()
        token=re.search('name="reset-token" content="([^"]+)"',client.get('/').text)[1]
        headers={'X-Reset-Token':token}
        self.assertEqual(client.post('/api/calibration/mag').status_code,403)
        self.assertEqual(client.get('/api/calibration/mag').status_code,405)
        self.assertEqual(client.post('/api/calibration/mag',headers=headers).status_code,409)
        self.gyro();state.connected=True
        self.assertEqual(client.post('/api/calibration/mag',headers=headers).status_code,200)
        self.assertEqual(client.get('/api/state').json['calibration']['phase'],'mag')
        self.assertEqual(client.post('/api/reset',headers=headers).status_code,409)
        self.assertEqual(client.post('/api/calibration/annulla',headers=headers).status_code,200)
        self.assertEqual(self.c.phase,'annullato')
        self.assertEqual(client.post('/api/calibration/riprova',headers=headers).status_code,200)
        self.assertEqual(self.c.phase,'mag')
        self.assertEqual(client.post('/api/calibration/unknown',headers=headers).status_code,409)


if __name__ == '__main__':
    unittest.main()
