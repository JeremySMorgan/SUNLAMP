import datetime
import numpy as np
import inspect
from src.utils import config

class Logger:

    class_logging_levels = [
        config.SYS_RUNNER_VERBOSITY,
        config.HL_TRAJ_VERBOSITY,
        config.STEPSEQ_VERBOSITY,
        config.MPLANNER_VERBOSITY,
        config.CLOOP_VERBOSITY
    ]

    class colors:
        '''Colors class:
        reset all colors with colors.reset
        two subclasses fg for foreground and bg for background.
        use as colors.subclass.colorname.
        i.e. colors.fg.red or colors.bg.green
        also, the generic bold, disable, underline, reverse, strikethrough,
        and invisible work with the main class
        i.e. colors.bold
        '''
        reset = '\033[0m'
        bold = '\033[01m'
        disable = '\033[02m'
        underline = '\033[04m'
        reverse = '\033[07m'
        strikethrough = '\033[09m'
        invisible = '\033[08m'

        class fg:
            black = '\033[30m'
            red = '\033[31m'
            green = '\033[32m'
            orange = '\033[33m'
            blue = '\033[34m'
            purple = '\033[35m'
            cyan = '\033[36m'
            lightgrey = '\033[37m'
            darkgrey = '\033[90m'
            lightred = '\033[91m'
            lightgreen = '\033[92m'
            yellow = '\033[93m'
            lightblue = '\033[94m'
            pink = '\033[95m'
            lightcyan = '\033[96m'

        class bg:
            black = '\033[40m'
            red = '\033[41m'
            green = '\033[42m'
            orange = '\033[43m'
            blue = '\033[44m'
            purple = '\033[45m'
            cyan = '\033[46m'
            lightgrey = '\033[47m'


    COLORS = {  # Unix codes for special priting
        "HEADER"	: 	{"value" :'\033[95m'},
        "OKBLUE" 		: 	{"value" :'\033[94m'},
        "OKGREEN" 		: 	{"value" :'\033[92m'},
        "BLUE" 		    : 	{"value" :'\033[34m'},
        # "ORANGE" 		: 	{"value" :'\033[43m'},
        "ORANGE" 		: 	{"value" :'\033[95m'},
        "WARNING" 		: 	{"value" :'\033[95m'},
        "FAIL" 			: 	{"value" :'\033[91m'},
        "ENDC" 			: 	{"value" :'\033[0m'},
        "BOLD" 			: 	{"value" :'\033[1m'},
        "UNDERLINE" 	: 	{"value" :'\033[4m'},
        "STANDARD"		:	{"value" :''}
    }

    @staticmethod
    def get_file_save_name():
        now = datetime.datetime.now()
        return now.strftime("%m-%d-%Y_%H:%M")

    @staticmethod
    def bold_txt(msg, color="STANDARD"):
        if not config.LOGGER_COLORS_ENABLED:
            return str(msg)
        else:
            return Logger.COLORS[color]["value"] + Logger.COLORS["BOLD"]["value"] + str(msg) + Logger.COLORS["ENDC"]["value"]

    @staticmethod
    def log_boolean(bool, invert=False, msg=None):
        success = "OKGREEN"
        fail = "FAIL"
        if invert:
            success = "FAIL"
            fail = "OKGREEN"
        pre = ""
        if bool == True:
            pre = Logger.COLORS[success]["value"]
            if msg is None:
                msg = "True"
            # return logger.COLORS[success]["value"] + "True" + logger.COLORS["ENDC"]["value"]
        elif bool == False:
            pre = Logger.COLORS[fail]["value"]
            if msg is None:
                msg = "False"
            # return logger.COLORS[fail]["value"] + "False" + logger.COLORS["ENDC"]["value"]
        else:
            pre = ""

        if config.LOGGER_COLORS_ENABLED:
            return pre + msg + Logger.COLORS["ENDC"]["value"]
        return msg

    @staticmethod
    def log_bold(s):

        if not config.LOGGER_COLORS_ENABLED:
            return str(s)
        else:
            return Logger.styled_text(str(s), "BOLD")

    @staticmethod
    def fail(msg):
        if not config.LOGGER_COLORS_ENABLED:
            return msg
        return Logger.COLORS["FAIL"]["value"]+msg+Logger.COLORS["ENDC"]["value"]

    @staticmethod
    def success(msg):
        return Logger.COLORS["OKGREEN"]["value"]+msg+Logger.COLORS["ENDC"]["value"]

    @staticmethod
    def styled_text(msg, style):
        if config.LOGGER_COLORS_ENABLED:
            return Logger.COLORS[style.upper()]["value"] + msg + Logger.COLORS["ENDC"]["value"]
        return msg

    @staticmethod
    def log_msg_tf(class_id, msg_type):

        if class_id < 0 or msg_type < 0:
            return True

        logging_level = Logger.class_logging_levels[class_id]

        # No output
        if logging_level == 0:
            return False

        # only fatal errors
        if logging_level == 1:
            if msg_type == 1:
                return True
            return False

        # fatal errors + initialization info
        if logging_level == 2:
            if msg_type <= 1:
                return True
            return False

        # 3 - fatal errors + initialization info + general info/status
        if logging_level == 3:
            if msg_type <= 2:
                return True
            return False

        # 3 - fatal errors + initialization info + general info/status + detailed info
        if logging_level == 4:
            return True

        print("unrecognized logging_level:",logging_level,"\t for class:",class_id)
        return True

    @staticmethod
    def log(message, color=None, class_id=-1, msg_type=-1):

        # class_id's:
        # 0 - system_runner
        # 1 - high_level_trajectory_generator.py
        # 2 - step_sequence_generator.py
        # 3 - motion_planner
        # 4 - control_loop

        # msg_types:
        # 0: initialization info
        # 1: fatal error
        # 2: info
        # 3: detailed info

        if not Logger.log_msg_tf(class_id, msg_type):
            return

        stack = inspect.stack()
        use_stack = True
        class_, method_ = "", ""
        caller = "__main__"
        try:
            class_ = stack[1][0].f_locals["self"].__class__.__name__
            method_ = stack[1][0].f_code.co_name
        except KeyError:
            use_stack = False
        if use_stack:
            caller = class_+"."+method_+"()"

        time = datetime.datetime.now()
        if color not in Logger.COLORS or color is None:
            color = Logger.COLORS.get("STANDARD").get("value")

        prefix = "["+str(time.day)+"/"+str(time.month)+ "/" + str(time.year) + " " + str(time.hour) + ":" + str(time.minute) + ":" + str(time.second) +  " ] "

        if not config.LOGGER_COLORS_ENABLED:
            print(prefix + "" + caller + ": " + message)
        else:
            prefix = prefix + Logger.COLORS["BOLD"]["value"]+ Logger.COLORS["UNDERLINE"]["value"]+ caller+ Logger.COLORS["ENDC"]["value"]+ ":"
            try:
                print_Str = prefix + ""+Logger.COLORS[color]["value"] + " "+message + " "+Logger.COLORS["ENDC"]["value"]
            except KeyError:
                print_Str = prefix + "" + Logger.COLORS["STANDARD"]["value"] + " " + message + " " + Logger.COLORS["ENDC"]["value"]
            print(print_Str)

    @staticmethod
    def log_hash(x):
        return str(hash(str(x)))[-5:]

    @staticmethod
    def pp_list(input_list_, round_amt=3):
        ret = "[ "
        if input_list_ is None:
            if not config.LOGGER_COLORS_ENABLED:
                return "None"
            return Logger.COLORS["BOLD"]["value"] + "None" + Logger.COLORS["ENDC"]["value"]

        for i in input_list_:
            if type(i) == list or type(i) == tuple:
                ret += Logger.pp_list(i)    # recursion, lfg
            else:
                if isinstance(i, (int, float, complex)):
                    if i == int(i):
                        ret += str(i) + ", "
                    else:
                        if i == np.inf:
                            ret += "inf"
                        else:
                            # ret += "%.3f" % i
                            ret += str(round(i, round_amt))
                        ret += ", "
                else:
                    ret += str(i)+", "
        ret = ret[:-2]
        ret += " ]"
        return ret

    @staticmethod
    def pp_double(dbl, rnd_amt=3):
        if dbl:
            if dbl == np.inf:
                return "inf"
            try:
                if int(dbl) == dbl:
                    return str(int(dbl))
            except ValueError:
                return 'NaN'
            return str(round(dbl, rnd_amt))
        return "<none>"