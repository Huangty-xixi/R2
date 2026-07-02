import sys
path = r'E:\R2\RC26_H7_R2\RC26_H7_R2\user\src\app_zone1.c'

with open(path, 'rb') as f:
    raw = f.read()

# Unique anchor: combination of DIR_RED define + MIN_SCALE define
anchor = b'#define APP_ZONE1_GRAB_SWEEP_DIR_RED           (-1)  //'
idx = raw.find(anchor)
if idx < 0:
    print('MISS: anchor not found')
    sys.exit(1)

# Find the end of the MIN_SCALE line (skip past DIR_RED line, then MIN_SCALE line)
nl1 = raw.find(b'\n', idx)           # end of DIR_RED line
nl2 = raw.find(b'\n', nl1 + 1)       # end of MIN_SCALE line

print(f'DIR_RED at {idx}, MIN_SCALE line ends at {nl2}')

# Insert new macro between MIN_SCALE and next line
insert = b'\n#define APP_ZONE1_GRAB_DETECT_SLOW_FACTOR   (0.2f)  // V7: detect slow factor'
new_raw = raw[:nl2] + insert + raw[nl2:]

with open(path, 'wb') as f:
    f.write(new_raw)
print('OK: app_zone1.c — added APP_ZONE1_GRAB_DETECT_SLOW_FACTOR')
