import unittest
from fairino_slam.navigation.footprint import FootprintMode, FootprintRequest, FootprintSelector


class FootprintInterfaceTests(unittest.TestCase):
    def test_dynamic_unavailable_never_silently_uses_chassis(self):
        class Chassis:
            def compute(self, request):
                raise AssertionError('must not use chassis fallback')
        selector = FootprintSelector({FootprintMode.CHASSIS: Chassis()})
        with self.assertRaisesRegex(NotImplementedError, 'upper_body_projection'):
            selector.compute('upper_body_projection', FootprintRequest(1., {}))
        with self.assertRaises(ValueError):
            selector.compute('unknown', FootprintRequest(1., {}))


if __name__ == '__main__':
    unittest.main()
