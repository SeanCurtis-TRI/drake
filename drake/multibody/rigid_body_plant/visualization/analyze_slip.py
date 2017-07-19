#!/usr/bin/env python
import numpy as np
import optparse
import pylab as plt
import sys

"""
Reads a slip text file and reports the slip statistics

Slip file is a set of column data of the form:
timestamp, idA, idB, n_X n_Y n_Z, p_X p_Y p_Z, v_X v_Y v_Z

(Note: the (x, y, z) triples for a given value are *not* separated by commas.)

Where there is contact between geometries a and b at point (p_X, p_Y, p_Z)
along contact normal (n_X, n_Y, n_Z) such that the bodies have relative
velocity at the contact point of (v_X, v_Y, v_Z).
"""

# The number of comma-separated columns in the data file.
DATA_WIDTH = 6

def make_pair(a, b):
    '''
    Create a hashable/sortable tuple out of a pair of element ids (represented
    by very large ints).
    '''
    if (a < b):
        return (a, b)
    else:
        return (b, a)

def pair_string(pair):
    '''Create a string representation of an id pair.'''
    # Given that ids are *huge* numbers, we present just the lower four digits
    # for visual clarity. This does *not* guarantee unique strings for otherwise
    # unique pairs.
    return 'Pair(*%d, *%d)' % (pair[0] % 10000, pair[1] % 10000)

class SlipData:
    '''The representation of a single row of the slip data.

    This currently excludes the unused contact point value. Instances hash
    strictly by time and order strictly by time.
    '''
    def __init__(self, t, n, v):
        self.time = t
        self.normal = n
        # confirm normal is normalized
        self.normal /= np.sqrt(np.dot(n, n))
        self.velocity = v

    def __cmp__(self, other):
        return cmp(self.time, other.time)

    def __hash__(self):
        return hash(self.time)

    def get_slip_speed( self ):
        '''Report the tangential speed at the contact point.'''
        norm_vel = np.dot(self.normal, self.velocity) * self.normal
        tan_vel = self.velocity - norm_vel
        return np.abs(np.sqrt(np.dot(tan_vel, tan_vel)))

def vector_from_string(s):
    '''Turns a string with three white-space separated values and returns
    a "vector" -- a numpy array of size (3,).'''
    tokens = s.split()
    return np.array([float(x) for x in tokens])

def read_data(data_file):
    '''Reads a slip data file and returns a dictionary mapping pairs 
    to slip data instances.'''
    print "Reading", data_file
    data = {}
    with open(data_file, 'r') as f:
        line = f.readline()
        while line:
            value_str = line.strip().split(',')
            if (len(value_str) != DATA_WIDTH):
                print "Read a line with insufficient fields:", line
                sys.exit(1)
            t = float(value_str[0])
            pair = make_pair(int(value_str[1]), int(value_str[2]))
            n = vector_from_string(value_str[3])
            p = vector_from_string(value_str[4])
            v = vector_from_string(value_str[5])
            if not data.has_key(pair):
                data[pair] = []
            data[pair].append(SlipData(t, n, v))
            line = f.readline()
    return data

def main():
    parser = optparse.OptionParser()
    parser.add_option('-i', '--input', dest='in_file', default='',
                      help='The path to the slip data file', type=str)
    parser.add_option('-b', '--bar', dest='bar',
                      help='If non-negative, draws a horizontal line on the '
                           'graph at the given value', default=-1, type=float)
    parser.add_option('-t', '--title', dest='title', default='',
                      help='The optional title to apply to the graph.')

    options, args = parser.parse_args()

    if not options.in_file:
        print "Missing input file!\n"
        parser.print_help()
        sys.exit(1)

    bar = options.bar
    title = options.title

    # A dictionary mapping the element pair to the contact data for it.
    data = read_data(options.in_file)
    plt.figure()
    legend_data = []
    minT = np.infty
    maxT = -minT
    for pair, data in data.items():
        # NOTE: this assumes that all times are *unique* but that they aren't
        # necessarily presented in order.
        data.sort()
        t = np.array([x.time for x in data])
        y = [x.get_slip_speed() for x in data]
        legend_data.append(pair_string(pair))
        plt.plot(t,y)
        if (t.min() < minT): minT = t.min()
        if (t.max() > maxT): maxT = t.max()
    plt.legend(legend_data)
    plt.xlim((minT, maxT))
    if (bar > 0):
        plt.plot((minT, maxT), (bar, bar), 'k')
    if (title):
        plt.title(title)
    plt.show()

if __name__ == '__main__':
    main()
