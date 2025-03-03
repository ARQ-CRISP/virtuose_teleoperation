#!/usr/bin/env python

import sys
from datetime import datetime

def generate_bagname(base_name):
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{base_name}_{current_time}"

if __name__ == "__main__":
    base_name = sys.argv[1] if len(sys.argv) > 1 else "stiffness_estimation_us"
    print(generate_bagname(base_name))
