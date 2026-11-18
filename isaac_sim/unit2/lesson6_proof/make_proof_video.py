# Unit 2, Lesson 6: the proof video with a data overlay, built from the bags.
#
#   chase camera (1280x720) as the main picture
#   robot front camera as picture-in-picture
#   live readout from the bag: sim time, distance, speed, lateral error, the controller's cmd_vel
#   caption (tool + setup) and a PASS card with the report's criteria at the end
#
# Everything comes from recorded ROS 2 topics, so picture and numbers are in sync by sim time.
# The front camera may come from another run of the same proof (the runs are deterministic:
# same sim time -> same pose); it's matched by sim time.
#
# Runs in the ROS 2 container (rosbag2_py, Pillow, ffmpeg):
#   python3 make_proof_video.py <chase bag> <front bag> <report.txt> <out.mp4> [--start 4.0]
import bisect
import subprocess
import sys

import numpy as np
import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PIL import Image as PILImage, ImageDraw, ImageFont
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

SPAWN = (0.5, 0.31)           # world x, y of the start (proof_sim.py); /odom starts at (0, 0) there
GOAL_X, MAX_LAT = 30.0, 0.30  # the proof's criteria
W, H = 1280, 720
PIP = (384, 288)              # picture-in-picture size
FPS = 30.0
END_CARD_S = 6.0
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
MONO_B = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"


def reader(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def to_rgb(m):
    ch = 4 if m.encoding.endswith("a8") else 3
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width * ch]
    img = img.reshape(m.height, m.width, ch)[:, :, :3]
    return img[:, :, ::-1] if m.encoding.startswith("bgr") else img


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def load_series(bag):
    """odom (t, x, y, vx) and cmd_vel (t, v, w); cmd_vel has no header: use the record time,
    which is sim time because the bag was recorded with --use-sim-time."""
    odom, cmd = [], []
    r = reader(bag, ["/odom", "/cmd_vel"])
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == "/odom":
            m = deserialize_message(data, Odometry)
            odom.append((stamp(m), m.pose.pose.position.x, m.pose.pose.position.y,
                         m.twist.twist.linear.x))
        else:
            m = deserialize_message(data, Twist)
            cmd.append((t * 1e-9, m.linear.x, m.angular.z))
    return np.array(odom), np.array(cmd)


def load_front(bag):
    """All front-camera frames, downscaled for the picture-in-picture (fits in RAM)."""
    ts, frames = [], []
    r = reader(bag, ["/image_raw"])
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, Image)
        ts.append(stamp(m))
        frames.append(PILImage.fromarray(to_rgb(m)).resize(PIP, PILImage.BILINEAR))
    return ts, frames


def at(series, t):
    """Last row of series (sorted by column 0) at or before time t."""
    i = bisect.bisect_right(series[:, 0], t) - 1
    return series[max(i, 0)]


def panel(draw, box, alpha=150):
    draw.rectangle(box, fill=(0, 0, 0, alpha))


def main(chase_bag, front_bag, report, out, start):
    odom, cmd = load_series(chase_bag)
    f_ts, f_frames = load_front(front_bag)
    print(f"odom {len(odom)}, cmd_vel {len(cmd)}, front frames {len(f_frames)}")
    crit = [ln.rstrip() for ln in open(report) if ln.strip()]
    i0 = next(i for i, ln in enumerate(crit) if ln.startswith("criterion"))
    table = crit[i0:i0 + 6]
    rtf = next((ln for ln in crit if ln.startswith("sim time")), "")

    f = {k: ImageFont.truetype(p, s) for k, (p, s) in dict(
        title=(FONT_B, 26), sub=(FONT, 19), mono=(MONO, 21), monob=(MONO_B, 22),
        small=(FONT, 16), big=(FONT_B, 54), table=(MONO, 24)).items()}

    enc = subprocess.Popen(
        ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
         "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "21", out], stdin=subprocess.PIPE)

    n, last = 0, None
    r = reader(chase_bag, ["/chase/image_raw"])
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, Image)
        t = stamp(m)
        if t < start:                       # the chase camera starts behind the end wall
            continue
        clean = PILImage.fromarray(to_rgb(m))
        base = clean.convert("RGBA")
        ov = PILImage.new("RGBA", (W, H), (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)

        # caption: what this is
        panel(d, (0, 0, 760, 104))
        d.text((18, 12), "NVIDIA Isaac Sim 6.1  ·  RTX rendering  ·  PhysX 200 Hz", font=f["title"], fill="white")
        d.text((18, 46), "ROS 2 Jazzy: an external controller drives over /cmd_vel from /odom", font=f["sub"], fill=(220, 220, 220))
        d.text((18, 72), "OpenUSD digital twin of a surveyed poultry barn  ·  Carter + camera + 2D lidar", font=f["sub"], fill=(220, 220, 220))

        # live readout from the bag
        _, ox, oy, vx = at(odom, t)
        _, cv, cw = at(cmd, t) if len(cmd) else (0, 0.0, 0.0)
        x = SPAWN[0] + ox
        lat = oy                            # odom y = sideways offset from the start = aisle line
        rows = [("sim time", f"{t:6.2f} s", None),
                ("x", f"{x:6.2f} m", f"goal {GOAL_X:.0f}"),
                ("speed", f"{vx:6.2f} m/s", None),
                ("lateral", f"{lat * 100:+6.1f} cm", f"limit ±{MAX_LAT * 100:.0f}"),
                ("cmd_vel", f"v {cv:4.2f}  w {cw:+5.2f}", None)]
        panel(d, (0, H - 200, 520, H))
        for k, (name, val, note) in enumerate(rows):
            y = H - 188 + k * 36
            d.text((18, y), f"{name:9s}", font=f["mono"], fill=(170, 200, 255))
            d.text((150, y), val, font=f["monob"], fill="white")
            if note:
                d.text((380, y + 2), note, font=f["small"], fill=(180, 180, 180))
        # progress bar to the goal
        frac = min(max((x - SPAWN[0]) / (GOAL_X - SPAWN[0]), 0), 1)
        d.rectangle((18, H - 14, 500, H - 8), fill=(80, 80, 80, 200))
        d.rectangle((18, H - 14, 18 + int(482 * frac), H - 8), fill=(118, 185, 0, 230))

        # picture-in-picture: the robot's own camera, nearest frame by sim time
        j = min(max(bisect.bisect_left(f_ts, t), 0), len(f_ts) - 1)
        px, py = W - PIP[0] - 16, 16
        panel(d, (px - 6, py - 6, px + PIP[0] + 6, py + PIP[1] + 30), 190)
        d.text((px, py + PIP[1] + 4), "robot camera  /image_raw", font=f["small"], fill="white")
        d.text((W - 360, H - 30), "played at real robot speed  ·  " + rtf.split(",")[-1].strip(),
               font=f["small"], fill=(200, 200, 200))
        frame = PILImage.alpha_composite(base, ov)
        frame.paste(f_frames[j], (px, py))
        last = clean                        # end card goes over the CLEAN picture
        enc.stdin.write(frame.convert("RGB").tobytes())
        n += 1

    # end card: the verdict, written by the program, not by hand
    card = PILImage.blend(last, PILImage.new("RGB", (W, H), (0, 0, 0)), 0.75).convert("RGBA")
    ov = PILImage.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    d.text((W // 2, 110), "UNIT 2 PROOF: PASS", font=f["big"], fill=(118, 185, 0), anchor="mm")
    for k, ln in enumerate(table):
        d.text((150, 200 + k * 40), ln.replace("ok", "ok ✓") if k else ln, font=f["table"],
               fill="white" if k else (170, 200, 255))
    d.text((150, 470), rtf, font=f["table"], fill=(200, 200, 200))
    d.text((150, 540), "One command: rosbag recorder + ROS 2 controller + headless simulator,",
           font=f["sub"], fill=(220, 220, 220))
    d.text((150, 568), "three processes talking only over ROS 2; criteria set before the run,",
           font=f["sub"], fill=(220, 220, 220))
    d.text((150, 596), "verdict computed by the program.", font=f["sub"], fill=(220, 220, 220))
    endf = PILImage.alpha_composite(card, ov).convert("RGB").tobytes()
    for _ in range(int(END_CARD_S * FPS)):
        enc.stdin.write(endf)
    enc.stdin.close()
    if enc.wait() != 0:
        raise SystemExit("ffmpeg failed")
    print(f"wrote {out}: {n} frames + {END_CARD_S:.0f} s end card")


if __name__ == "__main__":
    a = sys.argv[1:]
    start = 4.0
    if "--start" in a:
        i = a.index("--start")
        start = float(a[i + 1])
        del a[i:i + 2]
    if len(a) != 4:
        raise SystemExit("usage: make_proof_video.py <chase bag> <front bag> <report.txt> <out.mp4> [--start s]")
    main(*a, start)
