"""Check local Markdown targets in maintained onboarding guides (stdlib only)."""
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
PAGES = ['README.md', 'START_HERE_HE.md', 'CONTRIBUTING.md',
         'docs/README.md', 'docs/GETTING_STARTED.md', 'docs/CAPABILITIES.md',
         'docs/RECIPES.md', 'docs/RESULTS.md', 'docs/RESEARCH_EXAMPLES.md',
         'docs/TROUBLESHOOTING.md', 'docs/RELEASE_VALIDATION.md']
failures = []
checked = 0
for name in PAGES:
    page = ROOT / name
    for target in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)', page.read_text(encoding='utf-8')):
        target = target.split(' "')[0].strip('<>')
        parsed = urlsplit(target)
        if parsed.scheme or not parsed.path:
            continue
        checked += 1
        if not (page.parent / unquote(parsed.path)).is_file():
            failures.append(f'{name}: {target}')
if failures:
    raise SystemExit('Broken documentation links:\n' + '\n'.join(failures))
print(f'PASS: {checked} local links in {len(PAGES)} onboarding guides')
