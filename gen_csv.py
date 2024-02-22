"""Script to convert JLink RTT Log to valid csv"""

def main():
    """Main Function"""
    read = 0
    with open("test.log", 'r', encoding="UTF_8") as log_file:
        with open("op.csv", "w",encoding="UTF_8") as op:
            for line in log_file:
                if line.startswith("current"):
                    read = 1
                if read == 1 and line[0].isalpha() and not line.startswith("current"):
                    read = 0
                if read:
                    op.write(line)


if __name__ == "__main__":
    main()
