import sys


def parse_block(_c, _replacement_dict):
    for key in _replacement_dict:
        _c = _c.replace(key, _replacement_dict[key])
    return _c

def process_file(fname, replacement_dict):

    with open(fname, "r") as mesh:

        block_size = 10000

        c = mesh.read(int(block_size/2))
        c = parse_block(c, replacement_dict)
        sys.stdout.write(c)

        while True:

            c = mesh.read(block_size)
            if c == '':
                return

            c = parse_block(c, replacement_dict)
            sys.stdout.write(c)


if __name__ == '__main__':

    # python fix_broken_pcd.py file.mesh > file-1.mesh

    replacement_dict = {}

    for i in range(20):
        for preceding in range(10):
            if i == 0:
                bad_chars = str(preceding) + "-0."
                corr_chars = str(preceding) + "\n" + "-0."
            else:
                bad_chars = str(preceding) + str(-1*i) + "."
                corr_chars = str(preceding) + "\n" + str(-1*i) + "."

            replacement_dict[bad_chars] = corr_chars
            # print(f" bad chars:  '{bad_chars}' fixed: '{corr_chars}' ")
            # print(f" bad chars:  '{bad_chars}'")

    for preceding in range(1, 10):
        bad_chars = str(preceding) + "0."
        corr_chars = str(preceding) + "\n0."
        replacement_dict[bad_chars] = corr_chars

    # Assuming no x value greater than 9
    for preceding in range(1, 10):
        for char in ["1.", "2.", "3.", "4.", "5.", "6.", "7.", "8."]:
            bad_chars = str(preceding) + char
            corr_chars = str(preceding) + "\n" + char
            replacement_dict[bad_chars] = corr_chars

    replacement_dict = {}
    replacement_dict[" 1\n0."] = " 10."
    replacement_dict[" -1\n0."] = " -10."
    replacement_dict[" -1\n1."] = " -11."
    replacement_dict[" -1\n2."] = " -12."
    replacement_dict[" -1\n3."] = " -13."
    replacement_dict[" -1\n4."] = " -14."

    process_file(sys.argv[1], replacement_dict)
