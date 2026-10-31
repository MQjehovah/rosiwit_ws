#!/usr/bin/env python3
"""Fix rosbags-convert output metadata.yaml for rosbag2 Humble.

`rosbags-convert` (>=0.10) writes two things that rosbag2 Humble's metadata
parser rejects:

  * ``offered_qos_profiles: []``  (Humble expects a string, not a sequence)
  * ``type_description_hash:`` followed by an indented value on the next line
    (yaml-cpp sees an empty scalar -> "bad conversion")

Usage:
    python3 fix_rosbag2_metadata.py <bag_dir_or_metadata.yaml>
"""

import os
import re
import sys


def fix(path: str) -> None:
    if os.path.isdir(path):
        path = os.path.join(path, "metadata.yaml")
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    text = re.sub(
        r"type_description_hash:\s*\n\s+(\S+)",
        r"type_description_hash: \1",
        text,
    )
    text = text.replace("offered_qos_profiles: []", "offered_qos_profiles: ''")

    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print("fixed:", path)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    fix(sys.argv[1])
