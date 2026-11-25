"""Second half of test_height_mqtt_host.c: decode the firmware's ramp
records with dashboard/server.py and check they match the console text."""
import importlib.util
import pathlib
import sys

server_py = pathlib.Path(__file__).resolve().parents[2] / 'dashboard' / 'server.py'
spec = importlib.util.spec_from_file_location('server', server_py)
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)

ok = True
count = 0
for line in sys.stdin:
    rise, peak, hump, *raw = line.split()
    value = server.decode_packet(bytes(map(int, raw)), {})
    dash = [f"{value[k] * 100:.1f}" for k in ('rise_m', 'terrain_peak_m', 'hump_m')]
    console = [f"{float(x) * 100:.1f}" for x in (rise, peak, hump)]
    same = dash == console
    ok &= same
    count += 1
    print(f"console rise={rise:>7} peak={peak} hump={hump} m | "
          f"dashboard {' / '.join(dash)} cm  {'OK' if same else 'MISMATCH'}")

# After a ramp record, a status frame's 7-bit hump must not replace it.
prev = server.decode_packet(bytes([6 | (0x10 << 3), 32 + 150, 150, 150]), {})
status = server.decode_packet(bytes([1, 15, 128, 127]), prev)
ok &= status['hump_m'] == 0.150
print('status frame keeps 8-bit hump:', 'OK' if status['hump_m'] == 0.150 else 'MISMATCH')

print(f"{count} cases:", 'ALL CONSISTENT' if ok and count else 'INCONSISTENT')
sys.exit(0 if ok and count else 1)
