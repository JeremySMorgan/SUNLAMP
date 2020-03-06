import time

from src.utils.logger import Logger
from src.utils.thread_wrapper import HL_Thread
from pynput import keyboard


class UserInput:

    update_delay = .005

    exit_key = 'e'
    print_robot_config_key = 'c'
    pause_control_loop_key = 'p'
    enter_manual_control_mode_key = 'm'

    def __init__(self):

        self.keyboard_listener_container_thread = None

        self.down_cpressed = False
        self.up_cpressed = False
        self.right_cpressed = False
        self.left_cpressed = False

        self.exit_pressed = False
        self.print_robot_config_key_pressed = False
        self.pause_control_loop_key_pressed = False
        self.enter_manual_control_mode_key_pressed = False

    def start(self):
        self.keyboard_listener_container_thread = HL_Thread(self.keyboard_logger_containter_thread_func, "keyboard logger container", pass_motion_thread=True)

    def shutdown(self):
        self.keyboard_listener_container_thread.shutdown()
        Logger.log(self.__class__.__name__, "Suspending user_input thread")
        time.sleep(self.update_delay)

    def keyboard_logger_containter_thread_func(self, T):
        kl = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        kl.start()
        while 1:
            if not T.is_alive():
                break
            time.sleep(self.update_delay)
        kl.stop()

    def on_press(self, key):
        if format(key) == "Key.up":
            self.up_cpressed = True
        elif format(key) == "Key.down":
            self.down_cpressed = True
        elif format(key) == "Key.left":
            self.left_cpressed = True
        elif format(key) == "Key.right":
            self.right_cpressed = True
        elif format(key) == self.get_char_from_type("exit"):
            self.exit_pressed = True
        elif format(key) == self.get_char_from_type("pause"):
            self.pause_control_loop_key_pressed = True
        elif format(key) == self.get_char_from_type("robot_config"):
            self.print_robot_config_key_pressed = True
        elif format(key) == self.get_char_from_type("manual_control"):
            self.enter_manual_control_mode_key_pressed = True
        else:
            pass

    def on_release(self, key):
        if format(key) == "Key.up":
            self.up_cpressed = False
        elif format(key) == "Key.down":
            self.down_cpressed = False
        elif format(key) == "Key.left":
            self.left_cpressed = False
        elif format(key) == "Key.right":
            self.right_cpressed = False
        elif format(key) == self.get_char_from_type("pause"):
            self.pause_control_loop_key_pressed = False
        elif format(key) == self.get_char_from_type("robot_config"):
            self.print_robot_config_key_pressed = False
        else:
            pass

    def skip_one_config_backward(self):
        if self.left_cpressed:
            self.left_cpressed = False
            return True

    def skip_one_config_forward(self):
        if self.right_cpressed:
            self.right_cpressed = False
            return True

    def exit(self):
        return self.exit_pressed

    def get_char_from_type(self,str_):
        ret = ''
        if str_ == "exit":
            ret = "u'"+UserInput.exit_key+"'"
        if str_ == "robot_config":
            ret = "u'"+UserInput.print_robot_config_key+"'"
        if str_ == "pause":
            ret = "u'"+UserInput.pause_control_loop_key+"'"
        if str_ == "manual_control":
            ret = "u'"+UserInput.enter_manual_control_mode_key+"'"
        return ret

    def enter_manual_control_mode(self):
        return self.enter_manual_control_mode_key_pressed

    def pause_control_loop(self):
        return self.pause_control_loop_key_pressed

    def print_robot_config(self):
        if self.print_robot_config_key_pressed:
            self.print_robot_config_key_pressed = False
            return True
        return False

    def get_direction(self, scalar):
        dx = 0.0
        dy = 0.0
        if self.move_back():
            dx += -1.0
        if self.move_forward():
            dx += 1.0
        if self.move_left():
            dy += 1.0
        if self.move_right():
            dy += -1.0
        return scalar*dx, scalar*dy

    def move_left(self):
        if self.left_cpressed:
            return True

    def move_right(self):
        if self.right_cpressed:
            return True

    def move_back(self):
        if self.down_cpressed:
            return True

    def move_forward(self):
        if self.up_cpressed:
            return True