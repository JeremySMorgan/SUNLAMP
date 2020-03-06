import numpy as np

class PyUtils:

    @staticmethod
    def decimal_range(start, stop, inc):
        i = start
        while i < stop:
            yield i
            i += inc

    @staticmethod
    def format_time(s: float):
        nb_mins = int(np.floor(s / 60))
        nb_secs = int(s - nb_mins * 60)
        if nb_secs >= 10:
            secs_str = str(nb_secs)
        elif nb_secs > 0:
            secs_str = "0" + str(nb_secs)
        else:
            secs_str = "00"
        if nb_mins == 0:
            return "00:" + str(secs_str)
        return str(nb_mins) + ":" + str(secs_str)
