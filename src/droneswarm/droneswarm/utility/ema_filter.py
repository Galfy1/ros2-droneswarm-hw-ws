
class EMAFilter:
    def __init__(self, alpha=0.15):
        self.alpha = alpha
        self.value = None

    def reset():
        self.value = None

    def update(self, new_value):
        if self.value is None:
            # First sample initializes the filter
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
