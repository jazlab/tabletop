"""Assemble a version's report/src/01_literature.md: the version's
synthesis (01_literature_synthesis.md) followed by the shared literature
notes in research/ (headings demoted one level)."""

import re
import sys

from .. import paths

NOTES = (("notes_unsupervised_rl.md",
          "Appendix A: Unsupervised RL, skill discovery and touch-based "
          "curiosity"),
         ("notes_skeletal_models.md",
          "Appendix B: Skeletal and musculoskeletal agents, DEP, goal "
          "babbling and developmental touch"))


def demote(text: str) -> str:
    """Add one ``#`` to every H1-H3 heading outside code fences."""
    out, fence = [], False
    for line in text.splitlines():
        if line.startswith("```"):
            fence = not fence
        if not fence and re.match(r"^#{1,3} ", line):
            line = "#" + line
        out.append(line)
    return "\n".join(out)


def assemble(version) -> None:
    src = paths.report_dir(version) / "src"
    parts = [(src / "01_literature_synthesis.md").read_text()]
    for f, title in NOTES:
        body = (paths.RESEARCH / f).read_text()
        body = re.sub(r"^# .*\n", "", body, count=1)  # drop the file title
        parts.append("\\pagebreak\n\n## " + title + "\n\n" + demote(body))
    (src / "01_literature.md").write_text("\n\n".join(parts))


if __name__ == "__main__":
    assemble(sys.argv[1] if len(sys.argv) > 1 else "1")
