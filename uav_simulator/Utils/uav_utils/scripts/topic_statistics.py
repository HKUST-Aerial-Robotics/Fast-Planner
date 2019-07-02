#!/usr/bin/env python

import argparse as ap
import argcomplete

def main(**args):
  pass

if __name__ == '__main__':
  parser = ap.ArgumentParser()
  parser.add_argument('positional', choices=['spam', 'eggs'])
  parser.add_argument('--optional', choices=['foo1', 'foo2', 'bar'])
  argcomplete.autocomplete(parser)
  args = parser.parse_args()
  main(**vars(args))