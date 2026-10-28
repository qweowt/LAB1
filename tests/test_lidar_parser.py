import unittest

from lidar_protocol import parse_lidar_frame


class LidarParserTests(unittest.TestCase):
    def test_timestamp_and_ranges(self):
        frame = parse_lidar_frame(b"12:34:56.78> 1000 2000 0 3500", "127.0.0.1", 2368)
        self.assertEqual(frame.timestamp, "12:34:56.78")
        self.assertEqual(frame.ranges_mm, (1000, 2000, 0, 3500))
        self.assertEqual(frame.source_port, 2368)

    def test_without_timestamp(self):
        frame = parse_lidar_frame(b"500 600 700")
        self.assertIsNone(frame.timestamp)
        self.assertEqual(frame.ranges_mm, (500, 600, 700))

    def test_leading_separator_without_timestamp(self):
        frame = parse_lidar_frame(b"> 500 500 501 502")
        self.assertIsNone(frame.timestamp)
        self.assertEqual(frame.ranges_mm, (500, 500, 501, 502))


    def test_values_are_already_millimeters(self):
        frame = parse_lidar_frame(b"> 16000 2500 500")
        self.assertEqual(frame.ranges_mm, (16000, 2500, 500))
    def test_rejects_more_than_360_measurements(self):
        packet = (" ".join(["1000"] * 361)).encode()
        with self.assertRaises(ValueError):
            parse_lidar_frame(packet)

    def test_rejects_non_numeric_values(self):
        with self.assertRaises(ValueError):
            parse_lidar_frame(b"12:34:56.78> 1000 nope 3000")


if __name__ == "__main__":
    unittest.main()
