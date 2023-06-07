import threading

class RepeatingTimer_old:
    counter: int = 0
    timer: threading.Timer
    is_running: bool = False

    def __init__(self, interval_ms, callback):
        self.interval = interval_ms / 1000.0
        self.callback = callback
        # self.timer: threading.Timer = None
        self.i_is_running = False

    def _run(self):
        if not self.is_running:
            return

        self.i_is_running = False
        self.callback(self.counter)
        self.counter += 1
        self.start()

    def start(self):
        if not self.i_is_running:
            self.timer = threading.Timer(self.interval, self._run)
            self.timer.start()
            self.i_is_running = True
            self.is_running = True

    def stop(self):
        if self.i_is_running:
            self.timer.cancel()
            self.i_is_running = False
            self.is_running = False

    def reset(self):
        self.stop()
        self.counter = 0
        self.start()


class RepeatingTimer:
    counter: int = 0
    timer: threading.Timer
    is_running: bool = False

    def __init__(self, interval_ms, callback):
        self.interval = interval_ms / 1000.0  # Convert milliseconds to seconds
        self.callback = callback

    def _run(self):
        self.callback(self.counter)
        self.counter += 1
        if self.is_running:
            self.timer = threading.Timer(self.interval, self._run)
            self.timer.start()

    def start(self):
        if not self.is_running:
            self.is_running = True
            self.timer = threading.Timer(self.interval, self._run)
            self.timer.start()

    def stop(self):
        if self.is_running:
            self.timer.cancel()
            self.is_running = False

    def reset(self):
        self.counter = 0

