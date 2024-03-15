"""Script to convert JLink RTT Log to valid csv"""
import sys

def main(argv):
    """Main Function"""
    read = 0
    with open("op.txt", 'r', encoding="UTF_8") as log_file:
        with open(argv[1], "w",encoding="UTF_8") as op:
            for line in log_file:
                if line.startswith("t,x"):
                    read = 1
                if read:
                    op.write(line)
                

if __name__ == "__main__":
    main(sys.argv)
