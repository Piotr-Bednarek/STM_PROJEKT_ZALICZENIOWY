import pathlib
import sys

import matplotlib.pyplot as plt
import pandas as pd


def load_numeric_series(csv_path: pathlib.Path):
    df = pd.read_csv(csv_path, sep=None, engine="python")
    numeric_df = df.select_dtypes(include=["number"])
    if numeric_df.empty:
        raise ValueError(f"No numeric columns found in {csv_path}")

    if numeric_df.shape[1] == 1:
        x = numeric_df.index.to_numpy()
        y = numeric_df.iloc[:, 0].to_numpy()
        x_label = "Sample"
        y_label = str(numeric_df.columns[0])
    else:
        x = numeric_df.iloc[:, 0].to_numpy()
        y = numeric_df.iloc[:, 1].to_numpy()
        x_label = str(numeric_df.columns[0])
        y_label = str(numeric_df.columns[1])
    return x, y, x_label, y_label


def plot_aprbs_time_series(csv_path: pathlib.Path):
    df = pd.read_csv(csv_path, sep=None, engine="python")
    required_cols = ["time", "filtered", "control"]
    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        raise ValueError(f"Missing columns in {csv_path}: {', '.join(missing)}")

    fig, ax_left = plt.subplots(figsize=(10, 6))
    ax_right = ax_left.twinx()

    ax_left.plot(df["time"], df["filtered"], label="filtered", color="#1f77b4")
    ax_right.plot(df["time"], df["control"], label="control", color="#d62728")

    ax_left.set_title(csv_path.name)
    ax_left.set_xlabel("time")
    ax_left.set_ylabel("filtered")
    ax_right.set_ylabel("control")

    lines_left, labels_left = ax_left.get_legend_handles_labels()
    lines_right, labels_right = ax_right.get_legend_handles_labels()
    ax_left.legend(lines_left + lines_right, labels_left + labels_right, loc="best")

    ax_left.grid(True, alpha=0.3)
    fig.tight_layout()
    plt.show()


def plot_skok_time_series(csv_path: pathlib.Path):
    df = pd.read_csv(csv_path, sep=None, engine="python")
    required_cols = ["time", "filtered", "control"]
    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        raise ValueError(f"Missing columns in {csv_path}: {', '.join(missing)}")

    fig, ax_left = plt.subplots(figsize=(10, 6))
    ax_right = ax_left.twinx()

    ax_left.plot(df["time"], df["filtered"], label="filtered", color="#1f77b4")
    ax_right.plot(df["time"], df["control"], label="control", color="#d62728")

    ax_left.set_title(csv_path.name)
    ax_left.set_xlabel("time")
    ax_left.set_ylabel("filtered")
    ax_right.set_ylabel("control")

    lines_left, labels_left = ax_left.get_legend_handles_labels()
    lines_right, labels_right = ax_right.get_legend_handles_labels()
    ax_left.legend(lines_left + lines_right, labels_left + labels_right, loc="best")

    ax_left.grid(True, alpha=0.3)
    fig.tight_layout()
    plt.show()


def main():
    repo_root = pathlib.Path(__file__).resolve().parents[1]
    files = [
        repo_root / "APRBS1.csv",
        repo_root / "SKOK_100-110_1.csv",
    ]

    for path in files:
        if not path.exists():
            raise FileNotFoundError(f"Missing file: {path}")
        if path.name == "APRBS1.csv":
            plot_aprbs_time_series(path)
            continue
        if path.name == "SKOK_100-110_1.csv":
            plot_skok_time_series(path)
            continue

        x, y, x_label, y_label = load_numeric_series(path)

        plt.figure(figsize=(10, 6))
        plt.plot(x, y, label=path.name)
        plt.title(path.name)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
