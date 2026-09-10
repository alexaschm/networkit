#!/usr/bin/env python3

import unittest

import networkit as nk


class TestSubgraphIsomorphism(unittest.TestCase):

    def setUp(self):
        # Pattern: 0 -- 1
        self.pattern = nk.Graph(2)
        self.pattern.addEdge(0, 1)

        # Target: 0 -- 1 -- 2
        self.target = nk.Graph(3)
        self.target.addEdge(0, 1)
        self.target.addEdge(1, 2)

    def testVF2Callback(self):
        matches = []

        def callback(match):
            matches.append(match)

        iso = nk.isomorphism.VF2(self.pattern, self.target)
        iso.setCallback(callback)
        iso.run()

        self.assertEqual(len(matches), 4)
        self.assertEqual(len(matches[0]), 2)
        self.assertEqual(len(matches[1]), 2)

    def testRINumberOfMatches(self):
        iso = nk.isomorphism.RI(self.pattern, self.target)
        iso.run()

        self.assertEqual(iso.numberOfMatches(), 4)
        self.assertTrue(iso.hasMatch())

    def testParallelRICallback(self):
        matches = []

        def callback(workerId, match):
            matches.append((workerId, match))

        iso = nk.isomorphism.ParallelRI(self.pattern, self.target)
        iso.setCallback(callback, parallel=True)
        iso.run()

        self.assertEqual(len(matches), 4)
        self.assertEqual(len(matches[0][1]), 2)

    def testCallbackReceivesIndependentMatches(self):
        pattern = nk.Graph(2)
        pattern.addEdge(0, 1)

        target = nk.Graph(3)
        target.addEdge(0, 1)
        target.addEdge(1, 2)

        matches = []

        def callback(match):
            matches.append(match)

        iso = nk.isomorphism.VF2(pattern, target)
        iso.setCallback(callback)
        iso.run()

        saved = [tuple(match) for match in matches]

        self.assertEqual(len(saved), 4)
        self.assertEqual(len({id(match) for match in matches}), 4)

    def testCallbackCanMutateReceivedMatch(self):
        pattern = nk.Graph(2)
        pattern.addEdge(0, 1)

        target = nk.Graph(3)
        target.addEdge(0, 1)
        target.addEdge(1, 2)

        matches = []

        def callback(match):
            match[0] = 999
            matches.append(tuple(match))

        iso = nk.isomorphism.VF2(pattern, target)
        iso.setCallback(callback)
        iso.run()

        self.assertEqual(len(matches), 4)

    def testCallbackCalledExactlyOncePerMatch(self):
        pattern = nk.Graph(2)
        pattern.addEdge(0, 1)

        target = nk.Graph(4)
        target.addEdge(0, 1)
        target.addEdge(1, 2)
        target.addEdge(2, 3)

        callback_count = 0

        def callback(match):
            nonlocal callback_count
            callback_count += 1

        iso = nk.isomorphism.VF2(pattern, target)
        iso.setCallback(callback)
        iso.run()

        self.assertEqual(callback_count, 6)

    def testCallbackCanBeCallableObject(self):
        pattern = nk.Graph(2)
        pattern.addEdge(0, 1)

        target = nk.Graph(3)
        target.addEdge(0, 1)
        target.addEdge(1, 2)

        class Callback:
            def __init__(self):
                self.matches = []

            def __call__(self, match):
                self.matches.append(tuple(match))

        callback = Callback()

        iso = nk.isomorphism.VF2(pattern, target)
        iso.setCallback(callback)
        iso.run()

        self.assertEqual(len(callback.matches), 4)

    def testCallbackLambda(self):
        pattern = nk.Graph(2)
        pattern.addEdge(0, 1)

        target = nk.Graph(2)
        target.addEdge(0, 1)

        matches = []

        iso = nk.isomorphism.VF2(pattern, target)
        iso.setCallback(lambda match: matches.append(tuple(match)))
        iso.run()

        self.assertEqual(len(matches), 2)

    #TODO This test causes an issue that needs to be investigated.
    def testCallbackRaisesOnFirstMatch(self):
        pattern = nk.Graph(2)
        pattern.addEdge(0, 1)

        target = nk.Graph(4)
        target.addEdge(0, 1)
        target.addEdge(1, 2)
        target.addEdge(2, 3)

        calls = 0

        def callback(match):
            nonlocal calls
            calls += 1
            raise RuntimeError("stop")

        iso = nk.isomorphism.RI(pattern, target)
        iso.setCallback(callback)

        with self.assertRaises(RuntimeError):
            iso.run()

        self.assertEqual(calls, 1)

    def testDifferentCallbacksDoNotShareState(self):
        pattern = nk.Graph(2)
        pattern.addEdge(0, 1)

        target = nk.Graph(3)
        target.addEdge(0, 1)
        target.addEdge(1, 2)

        first = []
        second = []

        def callback1(match):
            first.append(tuple(match))

        def callback2(match):
            second.append(tuple(match))

        iso1 = nk.isomorphism.VF2(pattern, target)
        iso2 = nk.isomorphism.VF2(pattern, target)

        iso1.setCallback(callback1)
        iso2.setCallback(callback2)

        iso1.run()
        iso2.run()

        self.assertEqual(len(first), 4)
        self.assertEqual(len(second), 4)
        self.assertEqual(first, second)

    def testCallbackKeepsObjectAlive(self):
        pattern = nk.Graph(2)
        pattern.addEdge(0, 1)

        target = nk.Graph(3)
        target.addEdge(0, 1)
        target.addEdge(1, 2)

        class Callback:
            def __init__(self):
                self.count = 0

            def __call__(self, match):
                self.count += 1

        callback = Callback()

        iso = nk.isomorphism.VF2(pattern, target)
        iso.setCallback(callback)

        del callback
        iso.run()

        self.assertEqual(
            iso.numberOfMatches(),
            4
        )

    def testParallelCallbackReceivesValidWorkerIds(self):
        pattern = nk.Graph(2)
        pattern.addEdge(0, 1)

        target = nk.Graph(3)
        target.addEdge(0, 1)
        target.addEdge(1, 2)

        calls = []

        def callback(workerId, match):
            calls.append((workerId, tuple(match)))

        iso = nk.isomorphism.ParallelRI(pattern, target)
        iso.setCallback(callback, parallel=True)
        iso.run()

        self.assertEqual(len(calls), 4)

        for workerId, match in calls:
            self.assertGreaterEqual(workerId, 0)
            self.assertLess(workerId, iso.numberOfWorkers())
            self.assertEqual(len(match), 2)

    def testParallelCallbackExceptionPropagates(self):
        pattern = nk.Graph(2)
        pattern.addEdge(0, 1)

        target = nk.Graph(4)
        target.addEdge(0, 1)
        target.addEdge(1, 2)
        target.addEdge(2, 3)

        calls = []

        def callback(workerId, match):
            calls.append((workerId, tuple(match)))
            raise RuntimeError("parallel callback failed")

        iso = nk.isomorphism.ParallelRI(pattern, target)
        iso.setCallback(callback, parallel=True)

        with self.assertRaises(RuntimeError):
            iso.run()

        self.assertGreaterEqual(len(calls), 1)

    def testParallelCallbackResultsMatchSerialResults(self):
        pattern = nk.Graph(3)
        pattern.addEdge(0, 1)
        pattern.addEdge(1, 2)

        target = nk.Graph(4)
        target.addEdge(0, 1)
        target.addEdge(1, 2)
        target.addEdge(2, 3)

        serial_matches = []
        parallel_matches = []

        def serial_callback(match):
            serial_matches.append(tuple(match))

        def parallel_callback(workerId, match):
            parallel_matches.append(tuple(match))

        serial = nk.isomorphism.RI(pattern, target)
        serial.setCallback(serial_callback)
        serial.run()

        parallel = nk.isomorphism.ParallelRI(pattern, target)
        parallel.setCallback(parallel_callback, parallel=True)
        parallel.run()

        self.assertEqual(
            set(serial_matches),
            set(parallel_matches)
        )
        self.assertEqual(
            len(serial_matches),
            len(parallel_matches)
        )
    


if __name__ == "__main__":
    unittest.main()