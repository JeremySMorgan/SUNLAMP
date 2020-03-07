import pickle


class OutputSuperclass:

    UNFILLED = -1

    def __init__(self, output_type):
        self.output_type = output_type
        self.failed = False
        self.runtime = -1

    def save(self, save_file, print_=True):
        pickle.dump(self, open(save_file + ".pickle", "wb"))
        if print_:
            print(f"{self.output_type} output saved")

    def print_stats(self):
        pass