
def init(state):
    global __reload
    __reload = state


def set_reload(state):
    global __reload
    __reload = state


def get_reload():
    global __reload
    return __reload
