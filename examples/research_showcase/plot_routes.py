"""Plot archived TCP paths and their recorded objective values, without planning."""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent


def main():
    catalog = json.loads((HERE / 'catalog.json').read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=[r['id'] for r in catalog], default='kitchen')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    row = next(r for r in catalog if r['id'] == args.scene)
    root = HERE / row['routes_directory']
    fig = plt.figure(figsize=(12, 5), layout='constrained')
    axes = fig.add_subplot(121, projection='3d')
    costs = []
    colors = ['#277da8', '#38a169', '#d99025']
    labels = ['Shorter path', 'Less exposure', 'Better visibility']
    for i, (color, label) in enumerate(zip(colors, labels), 1):
        with np.load(root / f'route_{i:02d}.npz', allow_pickle=False) as data:
            ee = data['ee']
            axes.plot(*ee.T, color=color, label=label)
            axes.scatter(*ee[0], color=color, marker='o')
            axes.scatter(*ee[-1], color=color, marker='x')
            costs.append(data['costs'])
    axes.set(xlabel='TCP x (m)', ylabel='TCP y (m)', zlabel='TCP z (m)', title=row['title'])
    axes.legend(fontsize=8)
    values = np.asarray(costs)
    scale = np.max(values, axis=0)
    scale[scale == 0] = 1
    bars = fig.add_subplot(122)
    for i, (color, label) in enumerate(zip(colors, labels)):
        bars.bar(np.arange(3) + (i-1)*.25, values[i]/scale, width=.25, color=color, label=label)
    bars.set_xticks(range(3), ['TCP length', 'Exposure', 'Visibility loss'])
    bars.set(ylabel='Cost / largest of the three routes (lower is better)', ylim=(0, 1.15),
             title='Archived costs; exact values in metrics.csv')
    fig.suptitle('Saved 2026-11-21 preference routes; candidate-pool extrema, not a complete Pareto solve', fontsize=10)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)
    print(args.output)


if __name__ == '__main__':
    main()
