import os

def on_duckiebot():
    """ True if we are on the PI """
    import platform
    proc = platform.processor()
    on_the_duckiebot = not('x86' in proc)
    # armv7l
    return on_the_duckiebot

def on_laptop():
    return not on_circle() and not on_duckiebot()

def on_circle():
    return 'CIRCLECI' in os.environ