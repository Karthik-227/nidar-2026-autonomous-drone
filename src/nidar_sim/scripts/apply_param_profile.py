import argparse
import os
import re
import sys

DEFAULT_MASTER = 'tcp:127.0.0.1:5762'

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(os.path.dirname(HERE), 'config')

LINE = re.compile(r'^\s*([A-Z][A-Z0-9_]*)\s+(-?[0-9.]+)\s*$')

def read_profile(path):
    out = []
    with open(path) as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.split('#', 1)[0]
            if not line.strip():
                continue
            m = LINE.match(line)
            if not m:
                raise SystemExit(
                    f'{path}:{lineno}: not a NAME VALUE line: {raw.rstrip()!r}')
            out.append((m.group(1), float(m.group(2))))
    if not out:
        raise SystemExit(f'{path} defines no params.')
    return out

def profile_path(name):
    if os.path.isfile(name):
        return name
    path = os.path.join(CONFIG, f'nidar_{name}.parm')
    if not os.path.isfile(path):
        raise SystemExit(
            f'No profile {name!r}: expected {path} or a path to a .parm file.')
    return path

def fetch(conn, name, tries=4, timeout=1.5):
    import time
    for _ in range(tries):
        conn.mav.param_request_read_send(
            conn.target_system, conn.target_component, name.encode(), -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = conn.recv_match(type='PARAM_VALUE', blocking=True,
                                  timeout=max(0.1, deadline - time.time()))
            if msg is None:
                break
            if msg.param_id.strip('\x00') == name:
                return msg.param_value
    return None

def main():
    ap = argparse.ArgumentParser(
        description='Apply a nidar .parm profile to a running SITL over MAVLink, and verify it.')
    ap.add_argument('profile', nargs='?', default=None,
                    help="'baseline', 'tuning', or a path to a .parm file.")
    ap.add_argument('--master', default=DEFAULT_MASTER,
                    help=f'MAVLink endpoint (default {DEFAULT_MASTER}).')
    ap.add_argument('--show', action='store_true',
                    help='Only read the current values of both profiles\' '
                         'params and print them. Writes nothing.')
    args = ap.parse_args()

    if not args.show and args.profile is None:
        ap.error('give a profile name, or --show to read without writing.')

    try:
        from pymavlink import mavutil
    except ImportError:
        raise SystemExit('pymavlink is not installed: pip3 install pymavlink')

    if args.show:
        names = [n for n, _ in read_profile(profile_path('baseline'))]
        wanted = None
    else:
        wanted = read_profile(profile_path(args.profile))
        names = [n for n, _ in wanted]

    print(f'Connecting to {args.master} ...')
    try:
        conn = mavutil.mavlink_connection(args.master)
    except OSError as exc:
        raise SystemExit(
            f'Cannot reach {args.master}: {exc}. Is SITL running? '
            '(ps -ef | grep arducopter)')
    if conn.wait_heartbeat(timeout=20) is None:
        raise SystemExit(
            f'Connected to {args.master} but no heartbeat in 20 s. SITL may '
            'still be starting up.')
    print(f'Connected: system {conn.target_system}, '
          f'component {conn.target_component}')

    before = {n: fetch(conn, n) for n in names}

    if args.show:
        for n in names:
            v = before[n]
            print(f'  {n:<16}{"<unreadable>" if v is None else f"{v:g}"}')
        return 0

    missing = [n for n, v in before.items() if v is None]
    if missing:
        raise SystemExit(
            'These params could not be read back from the vehicle, so they do '
            'not exist in this build and writing them would do nothing '
            f'silently: {", ".join(missing)}')

    failed = []
    for name, value in wanted:
        conn.mav.param_set_send(
            conn.target_system, conn.target_component, name.encode(),
            float(value), conn.mav.MAV_PARAM_TYPE_REAL32 if hasattr(
                conn.mav, 'MAV_PARAM_TYPE_REAL32') else 9)
        got = fetch(conn, name)
        old = before[name]
        if got is None:
            failed.append((name, value, 'no read-back'))
            state = 'FAILED (no read-back)'
        elif abs(got - value) > max(1e-4, abs(value) * 1e-4):
            failed.append((name, value, f'reads {got:g}'))
            state = f'FAILED - reads {got:g}'
        else:
            state = 'ok' if abs(old - value) < 1e-9 else f'was {old:g}'
        print(f'  {name:<16}{value:<10g}{state}')

    if failed:
        print('\nSome writes did not land:', file=sys.stderr)
        for name, value, why in failed:
            print(f'  {name} -> {value:g}: {why}', file=sys.stderr)
        print('A value outside the param\'s own range is clamped, not '
              'rejected - ATC_ANGLE_MAX cannot go below 10 on Copter.',
              file=sys.stderr)
        return 1

    print(f'\n{os.path.basename(profile_path(args.profile))} applied and '
          f'verified. These are now in the eeprom, so they outrank any '
          f'--defaults file until changed again or the eeprom is wiped.')
    return 0

if __name__ == '__main__':
    sys.exit(main())
