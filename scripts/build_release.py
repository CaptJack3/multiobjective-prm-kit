"""Create a source ZIP without caches, virtualenvs or generated run directories.

Run from any directory: python scripts/build_release.py
The wheel/sdist can separately be built using: python -m build
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile


def main():
    root = Path(__file__).resolve().parents[1]
    version = re.search(r'__version__ = "([^"]+)"', (root / "src/prmkit/__init__.py").read_text()).group(1)
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=root / "dist" / f"PRMKIT_{version}_SOURCE.zip")
    args = parser.parse_args()
    excluded = {".git", ".venv", "__pycache__", ".pytest_cache", "build", "dist", "runs"}
    source_directories = {"src", "docs", "examples", "tests", "scripts", "slurm", "validation", ".github"}
    source_files = {".gitignore", "pyproject.toml", "MANIFEST.in", "README.md", "ARCHITECTURE.md",
                    "VALIDATION.md", "START_HERE_HE.md", "PROVENANCE.json", "THIRD_PARTY_NOTICES.md",
                    "requirements-tested.txt", "run_experiment.py", "CONTRIBUTING.md", "CHANGELOG.md", ".gitattributes"}
    files = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.resolve() == args.output.resolve():
            continue
        rel = path.relative_to(root)
        if (len(rel.parts) == 1 and rel.name not in source_files) or (len(rel.parts) > 1 and rel.parts[0] not in source_directories):
            continue
        if any(p in excluded or p.endswith(".egg-info") for p in rel.parts):
            continue
        if path.suffix in (".pyc", ".nbc", ".nbi"):
            continue
        files.append(path)
    prefix = f"multiobjective-prm-kit-{version}"
    manifest = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(".zip.tmp")
    with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for path in files:
            archive.write(path, f"{prefix}/{path.relative_to(root).as_posix()}")
        archive.writestr(f"{prefix}/SOURCE_MANIFEST.json", json.dumps(manifest, indent=2, sort_keys=True)+"\n")
    temporary.replace(args.output)
    with zipfile.ZipFile(args.output) as archive:
        if archive.testzip() is not None:
            raise RuntimeError("Archive CRC verification failed")
        for name, expected in manifest.items():
            if hashlib.sha256(archive.read(f"{prefix}/{name}")).hexdigest() != expected:
                raise RuntimeError(f"Archive hash mismatch: {name}")
    print(json.dumps(dict(archive=str(args.output), files=len(files), bytes=args.output.stat().st_size,
                         sha256=hashlib.sha256(args.output.read_bytes()).hexdigest()), indent=2))


if __name__ == "__main__":
    main()
