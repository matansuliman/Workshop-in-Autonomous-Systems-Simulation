import os, datetime
import csv
import pandas as pd
import matplotlib.pyplot as plt

from ...LowLevel.Utilities.globals import CONFIG, LOGGER, RUN_TIMESTAMP


class Log:
    """
    A flexible log with 'Time (sec)' as the x-axis and multiple y-axis channels.
    Each channel can be appended separately and plotted later.
    """

    def __init__(self, name, dir_name=None, ext=None, flush_interval=CONFIG["log"]["flush_interval"]):
        self._name = name
        self._flush_interval = flush_interval
        self._data = {"Time (sec)": []}
        self._sample_count = 0

        self._dir_name = dir_name or CONFIG["log"]["dir_name"]
        self._run_dir = os.path.join(self._dir_name, RUN_TIMESTAMP)
        os.makedirs(self._run_dir, exist_ok=True)

        self._plot_ext = ext or CONFIG["log"]["plot_ext"]
        self._flush_ext = ext or CONFIG["log"]["flush_ext"]
        self._plot_file_path = os.path.join(self._run_dir.__str__(), f"{self._name}.{self._plot_ext}")
        self._flush_file_path = os.path.join(self._run_dir.__str__(), f"{self._name}.{self._flush_ext}")


    # --- Time management ---
    def append_time(self, t: float):
        """Append a new timestamp."""
        self._data["Time (sec)"].append(t)

    # --- Channel management ---
    def _add_channel(self, channel_name: str):
        """Create a new channel if it doesn't exist."""
        if channel_name not in self._data:
            self._data[channel_name] = []

    def _append_channel(self, channel_name: str, value):
        """
        Append a single value to an existing or new channel.
        Useful when different modules add their own signals independently.
        """
        self._add_channel(channel_name)
        self._data[channel_name].append(value)

    def append_channels(self, **kwargs):
        """
        Append multiple channels at once.
        Example:
            log.append_channels(x=1.2, y=0.5)
        """
        for key, val in kwargs.items():
            self._append_channel(key, val)

        self._sample_count += 1
        if self._sample_count >= self._flush_interval:
            self._flush()

    # --- Queries ---
    def channels(self):
        """Return all Y-axis keys (excluding time)."""
        return [k for k in self._data if k != "Time (sec)"]

    # --- Plotting ---
    def plot(self, save=True, show=False):
        self._flush()

        df = pd.read_csv(self._flush_file_path)
        t = df["Time (sec)"]
        keys = [c for c in df.columns if c != "Time (sec)"]

        n_cols = 3
        n_rows = (len(keys) + n_cols - 1) // n_cols
        fig, axs = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 2.5 * n_rows))
        axs = axs.flatten()

        for i, key in enumerate(keys):
            axs[i].plot(t, df[key])
            axs[i].set_title(key)
            axs[i].grid(True)

        plt.tight_layout()
        if save:
            plt.savefig(self._plot_file_path)
            LOGGER.debug(f"{self._name} Plotted to {self._plot_file_path}")
        if show:
            plt.show()
        plt.close(fig)

    def __str__(self):
        return f"{self._name}, channels: {self._channels()}"

    def _flush(self):
        """Write current buffer to disk and clear memory."""
        if not any(len(v) > 0 for v in self._data.values()):
            LOGGER.debug(f"{self._name} skipped flushing  (no new data).")
            return

        keys = list(self._data.keys())
        rows = zip(*[self._data[k] for k in keys])

        file_exists = os.path.exists(self._flush_file_path)
        with open(self._flush_file_path, "a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(keys)  # header only once
            writer.writerows(rows)

        # reset in-memory data (keep structure)
        for k in keys:
            self._data[k].clear()

        self._sample_count = 0
        LOGGER.debug(f"{self._name} Flushed to {self._flush_file_path}")
