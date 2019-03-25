'''

Formats certificates in PEM for use in a C-style character array

'''

import sys

def fline(l):
    line = '"{}\\n"'.format(l.replace("\n", ""))
    return line

def main():

    if len(sys.argv) < 2:
        print("No certificate file specified")

    else:
        with open(sys.argv[1], "r") as f:

            for l in f.readlines():
                print(fline(l))

if __name__ == "__main__":
    main()
