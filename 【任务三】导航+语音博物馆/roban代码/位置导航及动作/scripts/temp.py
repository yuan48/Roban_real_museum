#!/usr/bin/env python
# coding=utf-8

from lejulib import *


def main():
    node_initial()

    try:
        client_walk.slow_walk('forward', 1)

    except Exception as err:
        serror(err)
    finally:
        finishsend()

if __name__ == '__main__':
    main()