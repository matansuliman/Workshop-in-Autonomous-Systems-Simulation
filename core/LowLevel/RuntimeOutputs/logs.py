import os
import matplotlib.pyplot as plt
from ...LowLevel.Utilities.globals import CONFIG, LOGGER


class Log:
    """
    A flexible log with 'Time (sec)' as the x-axis and multiple y-axis channels.
    Each channel can be appended separately and plotted later.
    """

    def __init__(self, name, dir_name=None, ext=None):
        self.name = name
        self.dir_name = dir_name or CONFIG["plotter"]["dir_name"]
        self.ext = ext or CONFIG["plotter"]["ext"]
        self.data = {"Time (sec)": []}

    # --- Time management ---
    def append_time(self, t: float):
        """Append a new timestamp."""
        self.data["Time (sec)"].append(t)

    # --- Channel management ---
    def add_channel(self, channel_name: str):
        """Create a new channel if it doesn't exist."""
        if channel_name not in self.data:
            self.data[channel_name] = []

    def append_channel(self, channel_name: str, value):
        """
        Append a single value to an existing or new channel.
        Useful when different modules add their own signals independently.
        """
        self.add_channel(channel_name)
        self.data[channel_name].append(value)

    def append_channels(self, **kwargs):
        """
        Append multiple channels at once.
        Example:
            log.append_channels(x=1.2, y=0.5)
        """
        for key, val in kwargs.items():
            self.append_channel(key, val)

    # --- Queries ---
    def channels(self):
        """Return all Y-axis keys (excluding time)."""
        return [k for k in self.data if k != "Time (sec)"]

    # --- Plotting ---
    def plot(self, save=True, show=False):
        """Plot all Y vs time."""
        keys = self.channels()
        if not keys:
            LOGGER.warning(f"Log {self.name}: no channels to plot.")
            return

        t = self.data["Time (sec)"]
        n_signals = len(keys)
        n_cols = 3
        n_rows = (n_signals + n_cols - 1) // n_cols

        fig, axs = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 2.5 * n_rows))
        axs = axs.flatten()

        for i, key in enumerate(keys):
            y = self.data[key]
            min_len = min(len(t), len(y))
            axs[i].plot(t[:min_len], y[:min_len])
            axs[i].set_title(key)
            axs[i].grid(True)

        plt.tight_layout()

        if save:
            os.makedirs(self.dir_name, exist_ok=True)
            filename = os.path.join(self.dir_name, f"{self.name}.{self.ext}")
            plt.savefig(filename)
            LOGGER.info(f"Log: plotted {self.name} -> {filename}")

        if show:
            plt.show()

        plt.close(fig)

    def __str__(self):
        return f"{self.name}, channels: {self.channels()}"
