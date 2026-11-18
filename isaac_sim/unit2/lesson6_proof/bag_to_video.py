# Unit 2, Lesson 6: turn the camera topic of a proof bag into an MP4.
#
# Runs in the ROS 2 container (rosbag2_py reads the bag, ffmpeg encodes), so the multi-GB bag
# never leaves the VM; only the small video does.
#   docker exec ros2 bash -c "source /opt/ros/jazzy/setup.bash && \
#     python3 /data/proof/bag_to_video.py /data/bags/proof_<RUN_ID> /data/proof/proof_<RUN_ID>.mp4"
#
# Playback speed: frames are encoded at the camera's rate IN SIM TIME (measured from the
# message stamps), so the video shows the robot at its real speed, whatever the RTF was.
import subprocess
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

TOPIC = "/image_raw"                       # default; --topic /chase/image_raw for the chase camera


def frames(bag):
    """Yield (stamp_s, HxWx3 uint8 RGB) for every image in the bag, in recorded order."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, Image)
        if m.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
            raise SystemExit(f"unsupported encoding {m.encoding}")
        ch = 4 if m.encoding.endswith("a8") else 3
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width * ch]
        img = img.reshape(m.height, m.width, ch)[:, :, :3]
        if m.encoding.startswith("bgr"):
            img = img[:, :, ::-1]                       # BGR -> RGB
        yield m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, img


def main(bag, out):
    it = frames(bag)
    first = [next(it) for _ in range(10)]              # measure the rate from the stamps
    rate = (len(first) - 1) / (first[-1][0] - first[0][0])
    h, w, _ = first[0][1].shape
    print(f"{TOPIC}: {w}x{h}, {rate:.1f} frames per sim second -> encoding at that rate")
    enc = subprocess.Popen(
        ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{w}x{h}", "-r", f"{rate:.3f}", "-i", "-",
         "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", out],
        stdin=subprocess.PIPE)
    n = 0
    for _, img in first:
        enc.stdin.write(img.tobytes())
        n += 1
    for _, img in it:
        enc.stdin.write(img.tobytes())
        n += 1
    enc.stdin.close()
    if enc.wait() != 0:
        raise SystemExit("ffmpeg failed")
    print(f"wrote {out}: {n} frames, {n / rate:.1f} s at robot speed")


if __name__ == "__main__":
    args = sys.argv[1:]
    if "--topic" in args:
        i = args.index("--topic")
        TOPIC = args[i + 1]
        del args[i:i + 2]
    if len(args) != 2:
        raise SystemExit("usage: bag_to_video.py <bag folder> <out.mp4> [--topic /chase/image_raw]")
    main(args[0], args[1])
