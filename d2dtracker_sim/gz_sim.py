#!/usr/bin/env python3

import os

def main():
    PX4_DIR = os.getenv('PX4_DIR')

    if PX4_DIR is not None:
        print(f'The value of PX4_DIR is {PX4_DIR}')
    else:
        print('PX4_DIR is not set')
        exit(1)

    cmd_str = "cd {} &&  make px4_sitl gz_x500_d435".format(PX4_DIR)
    os.system(cmd_str)

if __name__ == "__main__":
    main()
