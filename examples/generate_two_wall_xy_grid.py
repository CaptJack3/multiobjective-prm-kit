"""Enumerate two-wall XY geometry and previews, without loading a robot or PRM.

Run from the distribution directory:
  python examples/generate_two_wall_xy_grid.py examples/two_wall_xy_grid.yaml --plan
  python examples/generate_two_wall_xy_grid.py examples/two_wall_xy_grid.yaml --output runs/two_wall_xy_space

The grid contains wall segment endpoints, matching the historical grid convention.
Widths are segment lengths along X or Y; all walls stand on base_z_m. Pairs are
unordered, so exchanging wall names does not create another environment.
"""
from __future__ import annotations

import argparse
import html
import itertools
import json
import math
from pathlib import Path

import yaml


def number(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if positive and value <= 0:
        raise ValueError(f"{name} must be positive")
    return float(value)


def values(items, name, positive=False):
    if not isinstance(items, list) or not items:
        raise ValueError(f"{name} must be a nonempty list")
    result = [number(v, name, positive) for v in items]
    if len(set(result)) != len(result):
        raise ValueError(f"{name} contains duplicate values")
    return sorted(result)


def read_settings(path):
    cfg = yaml.safe_load(path.read_text())
    required = {'grid', 'widths_m', 'heights_m', 'thickness_m', 'orientations', 'base_z_m', 'exclude_wall_overlap'}
    if not isinstance(cfg, dict) or set(cfg) != required:
        raise ValueError(f"Configuration needs exactly these fields: {sorted(required)}")
    if not isinstance(cfg['grid'], dict) or set(cfg['grid']) != {'x_m', 'y_m'}:
        raise ValueError('grid needs x_m and y_m lists')
    cfg['grid'] = {key: values(cfg['grid'][key], key) for key in ('x_m', 'y_m')}
    for key in ('widths_m', 'heights_m'):
        cfg[key] = values(cfg[key], key, True)
    cfg['thickness_m'] = number(cfg['thickness_m'], 'thickness_m', True)
    cfg['base_z_m'] = number(cfg['base_z_m'], 'base_z_m')
    axes = cfg['orientations']
    if not isinstance(axes, list) or not axes or any(a not in ('X', 'Y') for a in axes) or len(set(axes)) != len(axes):
        raise ValueError('orientations must contain X, Y, or both, without duplicates')
    if not isinstance(cfg['exclude_wall_overlap'], bool):
        raise ValueError('exclude_wall_overlap must be true or false')
    return cfg


def wall_variants(cfg):
    axes = [cfg['grid']['x_m'], cfg['grid']['y_m']]
    walls = []
    for orientation, x, y, width, height in itertools.product(
            cfg['orientations'], *axes, cfg['widths_m'], cfg['heights_m']):
        axis = 0 if orientation == 'X' else 1
        end = [x, y]
        target = end[axis] + width
        endpoint = next((v for v in axes[axis] if abs(v - target) < 1e-9), None)
        if endpoint is None:
            continue
        end[axis] = endpoint
        wall = dict(candidate_id=f'wall_{len(walls):04d}', orientation=orientation,
                    start_xy_m=[x, y], end_xy_m=end, width_m=width, height_m=height,
                    position_m=[(x + end[0])/2, (y + end[1])/2, cfg['base_z_m'] + height/2],
                    dimensions_m=[width if axis == 0 else cfg['thickness_m'],
                                  width if axis == 1 else cfg['thickness_m'], height])
        walls.append(wall)
    missing = set(cfg['widths_m']) - {w['width_m'] for w in walls}
    if missing:
        raise ValueError(f'Widths {sorted(missing)} do not connect any two grid points')
    if len(walls) < 2:
        raise ValueError('The grid must produce at least two wall variants')
    return walls


def touching_or_overlapping(a, b):
    # Exact box test for the supported X/Y orientations, including touching.
    return all(abs(x-y) <= (da+db)/2 + 1e-10
               for x, y, da, db in zip(a['position_m'], b['position_m'], a['dimensions_m'], b['dimensions_m']))


def scene(index, pair):
    return dict(environment_id=f'env_{index:05d}', objects=[
        dict(name=f'wall_{i}', shape='box', position_m=w['position_m'],
             dimensions_m=w['dimensions_m'], collider=True)
        for i, w in enumerate(pair)])


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def figures(root, cfg, walls, pairs):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig, axes = plt.subplots(1, len(cfg['widths_m']), figsize=(6*len(cfg['widths_m']), 5), squeeze=False, layout='constrained')
    for ax, width in zip(axes.flat, cfg['widths_m']):
        seen = set()
        for w in walls:
            key = (tuple(w['start_xy_m']), tuple(w['end_xy_m']))
            if w['width_m'] != width or key in seen:
                continue
            seen.add(key)
            a, b = w['start_xy_m'], w['end_xy_m']
            ax.plot([a[0], b[0]], [a[1], b[1]], color='#247e96', lw=3, alpha=.5)
        points = np.array(list(itertools.product(cfg['grid']['x_m'], cfg['grid']['y_m'])))
        ax.scatter(points[:, 0], points[:, 1], c='#1e293b', s=22, zorder=3)
        example = next(w for w in walls if w['width_m'] == width)
        a, b = example['start_xy_m'], example['end_xy_m']
        ax.plot([a[0], b[0]], [a[1], b[1]], color='#c77d29', lw=6, zorder=4, label='One candidate wall')
        ax.scatter([a[0], b[0]], [a[1], b[1]], color='#c77d29', s=65, zorder=5)
        ax.set(title=f'Width {width:g} m | {len(seen)} placements', xlabel='X [m]', ylabel='Y [m]', aspect='equal')
        ax.set_xticks(cfg['grid']['x_m']); ax.set_yticks(cfg['grid']['y_m']); ax.grid(alpha=.2)
        ax.legend(loc='upper center', fontsize=9)
    fig.suptitle('Candidate wall placements | each segment is an alternative\nEvery placement has heights '+', '.join(f'{h:g} m' for h in cfg['heights_m']))
    fig.savefig(root/'candidate_grid.png', dpi=150)
    plt.close(fig)

    indices = sorted(set(np.linspace(0, len(pairs)-1, min(12, len(pairs)), dtype=int)))
    fig = plt.figure(figsize=(12, 3.5*math.ceil(len(indices)/3)), layout='constrained')
    faces = [[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
    corners = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])/2
    bounds = cfg['grid']; base = cfg['base_z_m']; height = max(cfg['heights_m'])
    for plot, index in enumerate(indices):
        ax = fig.add_subplot(math.ceil(len(indices)/3), 3, plot+1, projection='3d')
        for w, color in zip(pairs[index], ['#267b96', '#c77d29']):
            vertices = corners*np.array(w['dimensions_m'])+np.array(w['position_m'])
            ax.add_collection3d(Poly3DCollection([vertices[f] for f in faces], facecolors=color, edgecolors='#334155', linewidths=.4, alpha=.8))
        ax.set(xlim=(min(bounds['x_m'])-.04, max(bounds['x_m'])+.04),
               ylim=(min(bounds['y_m'])-.04, max(bounds['y_m'])+.04), zlim=(base, base+height+.03),
               xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]', title=f'env_{index:05d}')
        ax.view_init(elev=25, azim=-55)
    fig.suptitle(f'{len(indices)} of {len(pairs)} generated two-wall scenes | geometry only')
    fig.savefig(root/'examples_3d.png', dpi=130)
    plt.close(fig)


def generate(root, cfg, walls, pairs, summary, plots):
    from prmkit.artifacts import scene_preview
    root.mkdir(parents=True, exist_ok=False)
    (root/'settings.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    write_json(root/'wall_candidates.json', walls)
    catalog, cards = [], []
    for index, pair in enumerate(pairs):
        item = scene(index, pair)
        name = item['environment_id']
        folder = root/'environments'/name
        folder.mkdir(parents=True)
        write_json(folder/'scene.json', item)
        scene_preview(item, folder/'preview.svg')
        params = ' | '.join(f"{w['orientation']}: W={w['width_m']:g}, H={w['height_m']:g} m" for w in pair)
        catalog.append(dict(environment_id=name, wall_ids=[w['candidate_id'] for w in pair], scene=f'environments/{name}/scene.json'))
        cards.append(f'<article><h2>{name}</h2><img loading="lazy" src="environments/{name}/preview.svg" alt="{name} top view"><p>{html.escape(params)}</p><a href="environments/{name}/scene.json">Scene JSON</a></article>')
    write_json(root/'catalog.json', catalog)
    write_json(root/'summary.json', summary)
    extras = '<p><a href="candidate_grid.png">Candidate placement grid</a> · <a href="examples_3d.png">3D examples</a></p>' if plots else ''
    page = '<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Two-wall XY space</title><style>body{font:16px system-ui;margin:24px;background:#f3f6f8;color:#193042}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:16px}article{background:white;padding:16px;border-radius:8px}h2{font-size:18px}img{width:100%;height:230px;object-fit:contain}a{color:#086b83}</style>'
    page += f'<h1>Two-wall XY space: {len(pairs)} scenes</h1><p>Geometry only. No robot or PRM evaluation. XY previews do not show wall height.</p>'
    page += extras+'<div class="grid">'+''.join(cards)+'</div></html>'
    (root/'index.html').write_text(page)
    if plots:
        figures(root, cfg, walls, pairs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--plan', action='store_true', help='Show counts without writing files')
    parser.add_argument('--no-plots', action='store_true', help='Omit PNG figures; retain SVG previews and HTML gallery')
    args = parser.parse_args()
    cfg = read_settings(args.config)
    walls = wall_variants(cfg)
    pairs = [pair for pair in itertools.combinations(walls, 2)
             if not cfg['exclude_wall_overlap'] or not touching_or_overlapping(*pair)]
    summary = dict(stage='geometry_plan' if args.plan else 'generated_only', walls_per_scene=2,
                   wall_variants=len(walls), unordered_pairs_before_filter=math.comb(len(walls), 2),
                   excluded_touching_or_overlapping=math.comb(len(walls), 2)-len(pairs), environments=len(pairs),
                   prm_runs_executed=0, robot_feasibility_checked=False,
                   grid_convention='Wall segment endpoints lie on the XY grid; width is horizontal segment length.',
                   wall_pair_policy='Unordered pairs without replacement; touching counts as overlap.')
    if not pairs:
        raise ValueError('No two-wall scenes remain; enlarge the grid or change sizes')
    if not args.plan:
        if args.output is None:
            parser.error('--output is required unless --plan is used')
        generate(args.output, cfg, walls, pairs, summary, not args.no_plots)
        summary['output'] = str(args.output.resolve())
        summary['gallery'] = str((args.output/'index.html').resolve())
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
