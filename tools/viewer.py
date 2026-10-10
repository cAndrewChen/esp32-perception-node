"""Real-time 3D view of the tracked object, mapped into a global frame from the telemetry stream.

Usage:
    python tools/viewer.py --demo                 # synthetic packets, no simulator needed
    python tools/viewer.py                        # Wokwi: rfc2217://localhost:4000
    python tools/viewer.py --port COM5            # a real serial port
    python tools/viewer.py --file capture.bin     # a saved byte stream
    python tools/viewer.py --selftest             # checks the geometry, needs only numpy

Global frame (centimetres, origin at the sensor, which is fixed): X right, Y forward, Z up. Heading is
measured clockwise from +Y when seen from above, so it grows in the same direction as dx and the servo
angle in the firmware. The yaw at boot defines forward.

How a packet becomes a point:
    1. The object's direction in the camera frame comes from the pixel offset: 1 px = 1 deg (PX_PER_DEG,
       the same constant as firmware/main/world.h). Right and up are positive.
    2. Rotate it by the board's roll (about the camera axis), then pitch (about the right axis).
    3. Rotate by heading = IMU yaw + servo angle (the servo turns the camera on the base).
    4. Scale the unit direction by the sensor distance.
The ultrasonic sensor rides on the servo with the camera, so the distance is taken along the object's ray.
"""
import argparse
import collections
import math
import queue
import threading
import time

import numpy as np

from read_telemetry import Decoder, make_packet

PX_PER_DEG = 1.0


def rot_x(deg):   # pitch: positive tips the forward axis up
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(deg):   # roll: positive drops the right side
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z_cw(deg):  # heading: positive turns forward toward the right
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])


def base_rotation(p, flip_pitch=False):
    """Orientation of the board itself (IMU only, without the servo)."""
    pitch = -p["pitch"] if flip_pitch else p["pitch"]
    return rot_z_cw(p["yaw"]) @ rot_x(pitch) @ rot_y(p["roll"])


def camera_rotation(p, flip_pitch=False):
    """Orientation of the camera: the base turned further by the servo."""
    pitch = -p["pitch"] if flip_pitch else p["pitch"]
    return rot_z_cw(p["yaw"] + p["servo"]) @ rot_x(pitch) @ rot_y(p["roll"])


def object_position(p, flip_pitch=False):
    """Global position (cm) of the object, or None if it cannot be placed."""
    if not (p["obj_found"] and p["dist_valid"]):
        return None
    ax = math.radians(p["obj_x"] / PX_PER_DEG)     # right of the crosshair
    ay = math.radians(-p["obj_y"] / PX_PER_DEG)    # image y grows downward, so flip to "up"
    ray = np.array([math.tan(ax), 1.0, math.tan(ay)])
    ray /= np.linalg.norm(ray)
    return p["dist_cm"] * (camera_rotation(p, flip_pitch) @ ray)


def selftest():
    base = dict(obj_found=True, dist_valid=True, dist_fresh=True, dist_cm=100.0,
                pitch=0.0, roll=0.0, yaw=0.0, servo=0.0, obj_x=0.0, obj_y=0.0)

    def at(**kw):
        return object_position({**base, **kw})

    def near(a, b):
        return np.allclose(a, b, atol=1e-6)

    assert near(at(), [0, 100, 0]), "straight ahead"
    assert near(at(servo=90), [100, 0, 0]), "servo right 90 deg -> +X"
    assert near(at(yaw=-90), [-100, 0, 0]), "base yawed left 90 deg -> -X"
    assert near(at(yaw=40, servo=-40), [0, 100, 0]), "servo cancelling the base yaw -> straight ahead"
    assert near(at(pitch=30), [0, 100 * math.cos(math.radians(30)), 100 * math.sin(math.radians(30))]), "pitch up"
    assert near(at(obj_x=10), [100 * math.sin(math.radians(10)), 100 * math.cos(math.radians(10)), 0]), "object right of crosshair"
    assert at(obj_y=-10)[2] > 0, "object above crosshair is higher"
    assert at(roll=90, obj_x=10)[2] < 0, "rolling right-side-down 90 deg turns 'right' into 'down'"
    assert at(obj_found=False) is None and at(dist_valid=False) is None, "cannot place without object or distance"
    assert abs(np.linalg.norm(at(servo=33, pitch=12, roll=7, obj_x=9, obj_y=-4)) - 100.0) < 1e-6, "distance preserved"
    print("selftest ok: geometry checks pass")


class DemoSource:
    """Generates the byte stream the firmware would send, from a made-up spiral and a base that sways."""

    def __init__(self):
        self.t0 = time.time()
        self.buf = b""

    def read(self):
        time.sleep(0.02)
        t = time.time() - self.t0
        r = 40 + 15 * math.sin(0.15 * t)
        theta = 0.7 * t
        x_deg, y_deg = r * math.cos(theta), 0.5 * r * math.sin(theta)
        yaw = 25 * math.sin(0.2 * t)                      # base slowly swinging
        servo = max(-90, min(90, x_deg - yaw))            # an ideal tracker
        dist = 100 + 40 * math.sin(0.35 * t)
        pkt = make_packet(flags=7, t_ms=int(t * 1000), dist=int(dist * 10),
                          pitch=int(8 * math.sin(0.3 * t) * 100), roll=int(5 * math.sin(0.5 * t) * 100),
                          yaw=int(yaw * 100), servo=int(servo * 100), ox=0, oy=int(-y_deg * 100))
        return pkt


def open_source(args):
    if args.demo:
        return DemoSource().read
    if args.file:
        f = open(args.file, "rb")
        return lambda: (time.sleep(0.01), f.read(64))[1] or None
    import serial  # pip install pyserial
    ser = serial.serial_for_url(args.port or "rfc2217://localhost:4000", baudrate=115200, timeout=0.1)
    return lambda: ser.read(256)


def reader_thread(read, out_q, stats):
    dec = Decoder()
    while True:
        data = read()
        if data is None:
            break
        for p in dec.feed(data):
            out_q.put(p)
        stats.update(good=dec.good, bad=dec.bad_crc, skipped=dec.skipped)


def run_gui(args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    q = queue.Queue()
    stats = {}
    threading.Thread(target=reader_thread, args=(open_source(args), q, stats), daemon=True).start()

    fig = plt.figure(figsize=(9, 7.5))
    ax = fig.add_subplot(projection="3d")
    ax.set_xlabel("X right (cm)"); ax.set_ylabel("Y forward (cm)"); ax.set_zlabel("Z up (cm)")
    lim = [150.0]

    def set_limits():
        L = lim[0]
        ax.set_xlim(-L, L); ax.set_ylim(-L, L); ax.set_zlim(-L, L)
        ax.set_box_aspect((1, 1, 1))

    set_limits()
    ax.plot([0], [0], [0], "k^", ms=9, label="sensor (origin)")
    trail, = ax.plot([], [], [], "-", color="#2a78d6", lw=1.2, alpha=0.7, label="object trajectory")
    sphere, = ax.plot([], [], [], "o", ms=14, color="#2a78d6", label="object")
    ray, = ax.plot([], [], [], "--", color="#eb6834", lw=1.5, label="camera axis")
    axes_lines = [ax.plot([], [], [], "-", color=c, lw=2)[0] for c in ("#d03b3b", "#0ca30c", "#2a78d6")]
    ax.legend(loc="upper left", fontsize=8)
    info = fig.text(0.02, 0.02, "", family="monospace", fontsize=8, va="bottom")
    points = collections.deque(maxlen=args.trail)
    last = {}

    def update(_):
        got = False
        while True:
            try:
                p = q.get_nowait()
            except queue.Empty:
                break
            got = True
            last.update(p)
            pos = object_position(p, args.flip_pitch)
            points.append(pos if pos is not None else np.full(3, np.nan))   # NaN breaks the trail
        if not got or not last:
            return ()
        pos = object_position(last, args.flip_pitch)
        if len(points):
            arr = np.array(points)
            trail.set_data_3d(arr[:, 0], arr[:, 1], arr[:, 2])
        if pos is not None:
            sphere.set_data_3d([pos[0]], [pos[1]], [pos[2]])
            fresh = last["dist_fresh"]
            sphere.set_color("#2a78d6" if fresh else "#eba100")      # amber = held distance
            sphere.set_markerfacecolor("#2a78d6" if fresh else "none")
            far = float(np.max(np.abs(pos)))
            if far > lim[0] * 0.9:
                lim[0] = far * 1.3
                set_limits()
        else:
            sphere.set_data_3d([], [], [])
        cam = camera_rotation(last, args.flip_pitch) @ np.array([0, 1.0, 0])
        L = max(60.0, last["dist_cm"]) if last["dist_valid"] else 100.0
        ray.set_data_3d([0, cam[0] * L], [0, cam[1] * L], [0, cam[2] * L])
        R = base_rotation(last, args.flip_pitch)                  # board axes: right, forward, up
        for line, col in zip(axes_lines, R.T):
            line.set_data_3d([0, col[0] * 40], [0, col[1] * 40], [0, col[2] * 40])
        info.set_text(
            f"t={last['t_s']:7.2f}s  pitch={last['pitch']:6.1f} roll={last['roll']:6.1f} yaw={last['yaw']:6.1f} "
            f"servo={last['servo']:6.1f}\n"
            f"dist={'%.1f cm%s' % (last['dist_cm'], '' if last['dist_fresh'] else ' (held)') if last['dist_valid'] else 'no echo yet'}  "
            f"object={'(%.1f, %.1f) px' % (last['obj_x'], last['obj_y']) if last['obj_found'] else 'not found'}  "
            f"packets good={stats.get('good', 0)} bad_crc={stats.get('bad', 0)} skipped={stats.get('skipped', 0)}")
        return ()

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    ax.set_title("Tracked object in the global frame (board axes: red right, green forward, blue up)")
    plt.show()
    return anim


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--file")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--flip-pitch", action="store_true", help="use if the chip is mounted so that positive pitch is nose down")
    ap.add_argument("--trail", type=int, default=600, help="number of past points to keep")
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    run_gui(args)


if __name__ == "__main__":
    main()
