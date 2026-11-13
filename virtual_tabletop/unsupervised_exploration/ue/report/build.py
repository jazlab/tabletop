"""Build every PDF of a version's report.

usage (from unsupervised_exploration/):
    uv run --project .. python -m ue.report.figures VERSION
    uv run --project .. --with scikit-learn python -m ue.analysis.repertoire \
        VERSION
    uv run --project .. python -m ue.report.appendix VERSION
    uv run --project .. --with reportlab python -m ue.report.build VERSION

Sources are version_<v>/report/src/NN_*.md. ``{{generated_x}}``
placeholders are replaced by report/src/generated_x.md (written by the
figure/table generators or by hand). The literature document is assembled
from the version's synthesis plus the shared notes in research/. Output:
version_<v>/report/NN_*.pdf.
"""

import re
import sys

from .. import paths
from . import literature

DOCS = {
    "00_overview": "Reward-free exploration for the monkey arm: overview",
    "01_literature": "Part 1: Literature review",
    "02_environment": "Part 2: The play environment",
    "03_methods": "Part 3: Methods",
    "04_results": "Part 4: Results",
    "05_appendix": "Part 5: Appendix: runs, configs, videos",
}


def fill(text: str, src) -> str:
    """Replace ``{{generated_x}}`` with the contents of generated_x.md."""
    def sub(m):
        f = src / f"{m.group(1)}.md"
        return f.read_text() if f.exists() else f"*({m.group(1)} missing)*"
    return re.sub(r"\{\{(generated_[a-z0-9_]+)\}\}", sub, text)


def build(version, titles: dict | None = None) -> None:
    from . import md2pdf  # needs reportlab: uv run --with reportlab

    rdir = paths.report_dir(version)
    src = rdir / "src"
    # A version may define its own documents and titles.
    if titles is None and (src / "titles.yaml").exists():
        import yaml
        titles = yaml.safe_load((src / "titles.yaml").read_text())
    if (src / "01_literature_synthesis.md").exists():
        literature.assemble(version)
    for stem, title in (titles or DOCS).items():
        f = src / f"{stem}.md"
        if not f.exists():
            print("missing", f)
            continue
        tmp = src / f"_{stem}.filled.md"
        tmp.write_text(fill(f.read_text(), src))
        name = stem + ("_review" if stem == "01_literature" else "")
        md2pdf.main([str(tmp), str(rdir / f"{name}.pdf"), "--title", title])
        tmp.unlink()
        print("wrote", f"{name}.pdf")


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else "1")
