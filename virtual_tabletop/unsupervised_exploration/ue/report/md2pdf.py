"""Small, dependency-light Markdown-to-PDF converter built on reportlab.

Usage:
    python md2pdf.py IN.md OUT.pdf [--title "Document title"]

Supported Markdown subset: ATX headings (#, ##, ###), paragraphs with
**bold**, *italic*, `code`, [links](url) and $math$ (rendered as italic
text), bullet / numbered lists with nesting, GitHub pipe tables, fenced
code blocks, standalone images with captions, blockquotes, horizontal
rules and a ``\\pagebreak`` line. Image paths are resolved relative to the
Markdown file's directory.
"""

import argparse
import html
import re
import sys
from pathlib import Path

import matplotlib
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT, TA_RIGHT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import inch
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    HRFlowable,
    Image,
    KeepTogether,
    PageBreak,
    PageTemplate,
    Paragraph,
    Spacer,
    Table,
    TableStyle,
    XPreformatted,
)
from reportlab.platypus.tableofcontents import TableOfContents

# ---------------------------------------------------------------------------
# Page geometry and palette.
# ---------------------------------------------------------------------------
PAGE_W, PAGE_H = letter
MARGIN = 0.8 * inch
FRAME_W = PAGE_W - 2 * MARGIN
FRAME_H = PAGE_H - 2 * MARGIN
MAX_IMG_H = 0.6 * PAGE_H

TEXT = colors.HexColor('#0b0b0b')
MUTED = colors.HexColor('#52514e')
ACCENT = colors.HexColor('#2a78d6')
RULE = colors.HexColor('#d6d4cf')
CODE_BG = colors.HexColor('#f3f2ef')
ZEBRA = colors.HexColor('#f7f6f3')
HEAD_BG = colors.HexColor('#ebe9e4')

SANS = 'DejaVuSans'
MONO = 'DejaVuSansMono'
BODY_SIZE = 10
CODE_SIZE = 8.5
TABLE_SIZES = (9, 8.5, 8, 7.5, 7, 6.5, 6, 5.5)
CELL_PAD = 4
MIN_NOWRAP_SIZE = 7

# ---------------------------------------------------------------------------
# Fonts.
# ---------------------------------------------------------------------------


def register_fonts():
    """Registers DejaVu Sans / Sans Mono families from matplotlib's data."""
    ttf = Path(matplotlib.get_data_path()) / 'fonts' / 'ttf'
    for family in (SANS, MONO):
        variants = {
            family: f'{family}.ttf',
            f'{family}-Bold': f'{family}-Bold.ttf',
            f'{family}-Oblique': f'{family}-Oblique.ttf',
            f'{family}-BoldOblique': f'{family}-BoldOblique.ttf',
        }
        for name, fname in variants.items():
            pdfmetrics.registerFont(TTFont(name, str(ttf / fname)))
        pdfmetrics.registerFontFamily(
            family,
            normal=family,
            bold=f'{family}-Bold',
            italic=f'{family}-Oblique',
            boldItalic=f'{family}-BoldOblique',
        )


# ---------------------------------------------------------------------------
# Styles.
# ---------------------------------------------------------------------------


def make_styles():
    """Builds the paragraph styles used by the renderer.

    Returns:
        Dict mapping style names to ``ParagraphStyle`` objects.
    """
    base = ParagraphStyle(
        'body', fontName=SANS, fontSize=BODY_SIZE, leading=BODY_SIZE * 1.42,
        textColor=TEXT, spaceAfter=6, splitLongWords=1)
    s = {'body': base}
    s['h1'] = ParagraphStyle(
        'h1', parent=base, fontName=f'{SANS}-Bold', fontSize=21, leading=26,
        spaceBefore=4, spaceAfter=10, keepWithNext=1)
    s['h2'] = ParagraphStyle(
        'h2', parent=base, fontName=f'{SANS}-Bold', fontSize=14.5,
        leading=19, textColor=ACCENT, spaceBefore=14, spaceAfter=6,
        keepWithNext=1)
    s['h3'] = ParagraphStyle(
        'h3', parent=base, fontName=f'{SANS}-Bold', fontSize=11.5,
        leading=15, spaceBefore=10, spaceAfter=4, keepWithNext=1)
    s['h4'] = ParagraphStyle(
        'h4', parent=s['h3'], fontSize=10.5, textColor=MUTED)
    s['caption'] = ParagraphStyle(
        'caption', parent=base, fontName=f'{SANS}-Oblique', fontSize=8.5,
        leading=11.5, textColor=MUTED, alignment=TA_CENTER, spaceBefore=3,
        spaceAfter=10, leftIndent=24, rightIndent=24)
    s['quote'] = ParagraphStyle(
        'quote', parent=base, textColor=MUTED, spaceAfter=3)
    s['code'] = ParagraphStyle(
        'code', parent=base, fontName=MONO, fontSize=CODE_SIZE,
        leading=CODE_SIZE * 1.35, backColor=CODE_BG, borderPadding=6,
        leftIndent=6, rightIndent=6, spaceBefore=6, spaceAfter=12)
    s['toc_title'] = ParagraphStyle(
        'toc_title', parent=base, fontName=f'{SANS}-Bold', fontSize=9,
        textColor=MUTED, spaceBefore=4, spaceAfter=2)
    s['toc'] = ParagraphStyle(
        'toc', parent=base, fontSize=9.5, leading=13, spaceAfter=0,
        leftIndent=0, firstLineIndent=0)
    s['warn'] = ParagraphStyle(
        'warn', parent=base, textColor=colors.HexColor('#b3261e'))
    return s


# ---------------------------------------------------------------------------
# Inline formatting.
# ---------------------------------------------------------------------------
LATEX_SYMBOLS = {
    'alpha': 'α', 'beta': 'β', 'gamma': 'γ', 'delta': 'δ',
    'epsilon': 'ε', 'varepsilon': 'ε', 'zeta': 'ζ', 'eta': 'η',
    'theta': 'θ', 'iota': 'ι', 'kappa': 'κ', 'lambda': 'λ', 'mu': 'μ',
    'nu': 'ν', 'xi': 'ξ', 'pi': 'π', 'rho': 'ρ', 'sigma': 'σ',
    'tau': 'τ', 'phi': 'φ', 'varphi': 'φ', 'chi': 'χ', 'psi': 'ψ',
    'omega': 'ω', 'Gamma': 'Γ', 'Delta': 'Δ', 'Theta': 'Θ',
    'Lambda': 'Λ', 'Pi': 'Π', 'Sigma': 'Σ', 'Phi': 'Φ', 'Psi': 'Ψ',
    'Omega': 'Ω', 'times': '×', 'cdot': '·', 'pm': '±', 'leq': '≤',
    'le': '≤', 'geq': '≥', 'ge': '≥', 'neq': '≠', 'approx': '≈',
    'sim': '∼', 'infty': '∞', 'to': '→', 'rightarrow': '→',
    'leftarrow': '←', 'in': '∈', 'sum': 'Σ', 'prod': 'Π',
    'partial': '∂', 'nabla': '∇', 'propto': '∝', 'ldots': '…',
    'dots': '…', 'mid': '|',
}

_PH = '\x00{}\x00'
_PH_RE = re.compile('\x00(\\d+)\x00')
_ESCAPE_RE = re.compile(r'\\([\\`*_{}\[\]()#+\-.!$|>~])')
_CODE_RE = re.compile(r'(`+)(.+?)\1')
_MATH_RE = re.compile(r'(?<![\\$\w])\$(?=\S)([^$\n]+?)(?<=\S)\$(?![\w$])')
_IMG_INLINE_RE = re.compile(r'!\[([^\]]*)\]\(([^)\s]*)[^)]*\)')
_LINK_RE = re.compile(r'\[([^\]]+)\]\(\s*<?([^)\s>]*)>?(?:\s+"[^"]*")?\s*\)')
_AUTOLINK_RE = re.compile(r'<(https?://[^>\s]+)>')
_BARE_URL_RE = re.compile(
    r'(?<![\w/"=])(https?://[^\s<>()\x00]+[^\s<>().,;:!?\x00])')
_EMPHASIS = [
    (re.compile(r'\*\*\*(?=\S)(.+?)(?<=\S)\*\*\*'), r'<b><i>\1</i></b>'),
    (re.compile(r'\*\*(?=\S)(.+?)(?<=\S)\*\*'), r'<b>\1</b>'),
    (re.compile(r'(?<!\w)__(?=\S)(.+?)(?<=\S)__(?!\w)'), r'<b>\1</b>'),
    (re.compile(r'(?<![*\w])\*(?=[^\s*])(.+?)(?<=[^\s*])\*(?!\*)'),
     r'<i>\1</i>'),
    (re.compile(r'(?<!\w)_(?=[^\s_])(.+?)(?<=[^\s_])_(?!\w)'), r'<i>\1</i>'),
    (re.compile(r'~~(?=\S)(.+?)(?<=\S)~~'), r'<strike>\1</strike>'),
]


def xml_escape(text):
    """Escapes the XML special characters ``&``, ``<`` and ``>``."""
    return (text.replace('&', '&amp;').replace('<', '&lt;')
            .replace('>', '&gt;'))


def _attr(text):
    """Escapes a string for use inside a double-quoted XML attribute."""
    return xml_escape(text).replace('"', '%22')


def render_math(src):
    """Renders a ``$...$`` math snippet as reportlab italic markup.

    Supports Unicode input, a handful of LaTeX symbol macros (``\\alpha``,
    ``\\leq``, ...), and simple ``_x`` / ``_{xy}`` / ``^x`` / ``^{xy}``
    sub- and superscripts.

    Args:
        src: Raw text between the dollar signs.

    Returns:
        Reportlab paragraph markup.
    """
    src = re.sub(r'\\(mathrm|mathbf|mathit|text|operatorname)\{([^}]*)\}',
                 r'\2', src)
    src = re.sub(r'\\([A-Za-z]+)',
                 lambda m: LATEX_SYMBOLS.get(m.group(1), m.group(1)), src)
    src = src.replace('\\,', ' ').replace('\\;', ' ').replace('\\!', '')
    out = xml_escape(src)
    script = r'([_^])(?:\{([^}]*)\}|([^\s{}_^]))'

    def sub(m):
        tag = 'sub' if m.group(1) == '_' else 'super'
        body = m.group(2) if m.group(2) is not None else m.group(3)
        return f'<{tag}>{body}</{tag}>'

    out = re.sub(script, sub, out)
    out = out.replace('{', '').replace('}', '')
    return f'<i>{out}</i>'


def inline(text, link_color='#2a78d6'):
    """Converts inline Markdown to reportlab paragraph markup.

    Code spans, math, links and backslash escapes are first replaced by
    placeholders, the remaining text is XML-escaped and emphasis is
    applied, and finally the placeholders are restored.

    Args:
        text: Inline Markdown source.
        link_color: Hex colour for hyperlinks.

    Returns:
        A markup string safe to pass to ``Paragraph``.
    """
    store = []

    def stash(markup):
        store.append(markup)
        return _PH.format(len(store) - 1)

    def emphasis(s):
        for pattern, repl in _EMPHASIS:
            s = pattern.sub(repl, s)
        return s

    def link(label_markup, url):
        if url.startswith('#'):
            url = '#' + slugify(url[1:])
        return (f'<a href="{_attr(url)}" color="{link_color}">'
                f'{label_markup}</a>')

    text = _ESCAPE_RE.sub(lambda m: stash(xml_escape(m.group(1))), text)
    text = _CODE_RE.sub(
        lambda m: stash(
            f'<font name="{MONO}" size="{{size}}" backColor="#efeeea">'
            f'{xml_escape(m.group(2).strip())}</font>'), text)
    text = _MATH_RE.sub(lambda m: stash(render_math(m.group(1))), text)
    text = _IMG_INLINE_RE.sub(
        lambda m: stash(f'<i>[{xml_escape(m.group(1) or "image")}]</i>'),
        text)
    text = _LINK_RE.sub(
        lambda m: stash(link(emphasis(xml_escape(m.group(1))),
                             m.group(2))), text)
    text = _AUTOLINK_RE.sub(
        lambda m: stash(link(xml_escape(m.group(1)), m.group(1))), text)
    text = _BARE_URL_RE.sub(
        lambda m: stash(link(xml_escape(m.group(1)), m.group(1))), text)
    text = emphasis(xml_escape(text))
    for _ in range(10):
        new = _PH_RE.sub(lambda m: store[int(m.group(1))], text)
        if new == text:
            break
        text = new
    return text


def plain_text(text):
    """Strips inline Markdown, returning approximate visible text."""
    text = _LINK_RE.sub(r'\1', text)
    text = _IMG_INLINE_RE.sub(r'\1', text)
    text = _CODE_RE.sub(r'\2', text)
    text = _MATH_RE.sub(r'\1', text)
    for pattern, _ in _EMPHASIS:
        text = pattern.sub(r'\1', text)
    text = _ESCAPE_RE.sub(r'\1', text)
    return text.strip()


def slugify(text):
    """Builds a GitHub-style anchor slug for a heading."""
    text = plain_text(text).lower()
    text = re.sub(r'[^\w\- ]', '', text, flags=re.UNICODE)
    return re.sub(r'\s', '-', text.strip())


def make_para(markup, style, **kwargs):
    """Creates a Paragraph, falling back to escaped text on markup errors.

    Args:
        markup: Reportlab markup (``{size}`` is replaced by the code font
            size derived from ``style``).
        style: ``ParagraphStyle`` to use.
        **kwargs: Forwarded to ``Paragraph``.

    Returns:
        A ``Paragraph`` flowable.
    """
    code_size = round(style.fontSize * 0.9, 1)
    markup = markup.replace('size="{size}"', f'size="{code_size}"')
    try:
        return Paragraph(markup, style, **kwargs)
    except ValueError:
        stripped = re.sub(r'<[^>]+>', '', markup)
        stripped = xml_escape(html.unescape(stripped))
        return Paragraph(stripped, style, **kwargs)


# ---------------------------------------------------------------------------
# Block-level parsing.
# ---------------------------------------------------------------------------
_HEADING_RE = re.compile(r'^(#{1,6})\s+(.*?)\s*#*\s*$')
_FENCE_RE = re.compile(r'^(\s*)(`{3,}|~{3,})\s*([\w+\-.]*)')
_HR_RE = re.compile(r'^\s{0,3}([-*_])(\s*\1){2,}\s*$')
_LIST_RE = re.compile(r'^(\s*)([-*+]|\d{1,9}[.)])\s+(.*)$')
_QUOTE_RE = re.compile(r'^\s{0,3}>\s?(.*)$')
_IMG_BLOCK_RE = re.compile(
    r'^\s*!\[(.*?)\]\(\s*<?([^)\s>]+)>?(?:\s+"(.*?)")?\s*\)\s*$')
_TABLE_SEP_RE = re.compile(
    r'^\s*\|?\s*:?-+:?\s*(\|\s*:?-+:?\s*)*\|?\s*$')
_PAGEBREAK_RE = re.compile(r'^\s*\\(pagebreak|newpage)\s*$')
BULLETS = ('•', '◦', '▪')


def _split_row(line):
    """Splits a pipe-table row into stripped cell strings."""
    line = line.strip()
    if line.startswith('|'):
        line = line[1:]
    if line.endswith('|') and not line.endswith('\\|'):
        line = line[:-1]
    cells = re.split(r'(?<!\\)\|', line)
    return [c.strip().replace('\\|', '|') for c in cells]


def _is_table_start(lines, i):
    """Returns True if ``lines[i]`` begins a pipe table."""
    return (i + 1 < len(lines) and '|' in lines[i]
            and '|' in lines[i + 1] and '-' in lines[i + 1]
            and bool(_TABLE_SEP_RE.match(lines[i + 1])))


class Renderer:
    """Converts Markdown source lines into a list of reportlab flowables.

    Attributes:
        styles: Paragraph styles from ``make_styles``.
        base_dir: Directory used to resolve relative image paths.
        headings: List of ``(level, plain_text, key)`` for each heading.
    """

    def __init__(self, styles, base_dir):
        self.styles = styles
        self.base_dir = Path(base_dir)
        self.headings = []
        self._keys = set()

    # -- helpers -----------------------------------------------------------

    def _starts_block(self, lines, i):
        """Returns True if ``lines[i]`` starts a non-paragraph block."""
        line = lines[i]
        return bool(
            _HEADING_RE.match(line) or _FENCE_RE.match(line)
            or _HR_RE.match(line) or _LIST_RE.match(line)
            or _QUOTE_RE.match(line) or _IMG_BLOCK_RE.match(line)
            or _PAGEBREAK_RE.match(line) or _is_table_start(lines, i)
            or line.lstrip().startswith('<!--'))

    def _unique_key(self, text):
        base = slugify(text) or 'section'
        key, n = base, 1
        while key in self._keys:
            n += 1
            key = f'{base}-{n}'
        self._keys.add(key)
        return key

    # -- main loop ---------------------------------------------------------

    def render(self, lines, in_quote=False):
        """Renders Markdown lines to flowables.

        Args:
            lines: Source lines without trailing newlines.
            in_quote: Whether the lines are inside a blockquote.

        Returns:
            List of flowables.
        """
        out = []
        i, n = 0, len(lines)
        while i < n:
            line = lines[i]
            stripped = line.strip()
            if not stripped:
                i += 1
                continue
            if stripped.startswith('<!--'):
                while i < n and '-->' not in lines[i]:
                    i += 1
                i += 1
                continue
            if _PAGEBREAK_RE.match(line):
                out.append(PageBreak())
                i += 1
                continue
            m = _FENCE_RE.match(line)
            if m:
                i = self._code_block(lines, i, m, out)
                continue
            m = _HEADING_RE.match(line)
            if m:
                out.append(self._heading(len(m.group(1)), m.group(2),
                                         in_quote))
                i += 1
                continue
            if _HR_RE.match(line):
                out.append(HRFlowable(width='100%', thickness=0.6,
                                      color=RULE, spaceBefore=6,
                                      spaceAfter=10))
                i += 1
                continue
            if _is_table_start(lines, i):
                i = self._table(lines, i, out)
                continue
            m = _IMG_BLOCK_RE.match(line)
            if m:
                out.append(self._image(m.group(2), m.group(1)))
                i += 1
                continue
            if _QUOTE_RE.match(line):
                i = self._blockquote(lines, i, out)
                continue
            if _LIST_RE.match(line):
                i = self._list(lines, i, out, in_quote)
                continue
            i = self._paragraph(lines, i, out, in_quote)
        return out

    # -- blocks ------------------------------------------------------------

    def _heading(self, level, text, in_quote):
        style = self.styles[f'h{min(level, 4)}']
        key = self._unique_key(text)
        markup = f'<a name="{key}"/>{inline(text)}'
        para = make_para(markup, style)
        if level <= 3 and not in_quote:
            para.md_heading = (level, plain_text(text), key)
            self.headings.append(para.md_heading)
        return para

    def _paragraph(self, lines, i, out, in_quote):
        parts = []
        n = len(lines)
        while i < n and lines[i].strip():
            if parts and self._starts_block(lines, i):
                break
            raw = lines[i]
            hard = raw.endswith('  ') or raw.rstrip().endswith('\\')
            text = raw.strip()
            if hard and text.endswith('\\'):
                text = text[:-1]
            parts.append(inline(text) + ('<br/>' if hard else ''))
            i += 1
        markup = ' '.join(parts)
        markup = re.sub(r'(<br/>\s*)+$', '', markup)
        style = self.styles['quote' if in_quote else 'body']
        out.append(make_para(markup, style))
        return i

    def _code_block(self, lines, i, match, out):
        fence = match.group(2)
        indent = len(match.group(1))
        body = []
        i += 1
        while i < len(lines):
            if lines[i].strip().startswith(fence[0] * len(fence)) and \
                    set(lines[i].strip()) <= {fence[0]}:
                i += 1
                break
            line = lines[i].expandtabs(4)
            if indent and line[:indent].strip() == '':
                line = line[indent:]
            body.append(line.rstrip())
            i += 1
        style = self.styles['code']
        width = FRAME_W - style.leftIndent - style.rightIndent - 2
        char_w = pdfmetrics.stringWidth('M', MONO, style.fontSize)
        max_chars = max(20, int(width / char_w))
        wrapped = []
        for line in body or ['']:
            while len(line) > max_chars:
                wrapped.append(line[:max_chars])
                line = '  ' + line[max_chars:]
            wrapped.append(line)
        out.append(XPreformatted(xml_escape('\n'.join(wrapped)), style))
        return i

    def _image(self, src, alt):
        path = Path(src)
        if not path.is_absolute():
            path = self.base_dir / path
        if not path.exists():
            return make_para(
                f'[missing image: {xml_escape(src)}]', self.styles['warn'])
        try:
            reader = ImageReader(str(path))
            px_w, px_h = reader.getSize()
        except Exception as err:  # pylint: disable=broad-except
            return make_para(
                f'[unreadable image {xml_escape(src)}: '
                f'{xml_escape(str(err))}]', self.styles['warn'])
        dpi = _image_dpi(path)
        w, h = px_w * 72.0 / dpi, px_h * 72.0 / dpi
        scale = min(1.0, FRAME_W / w, MAX_IMG_H / h)
        img = Image(str(path), width=w * scale, height=h * scale)
        img.hAlign = 'CENTER'
        parts = [Spacer(1, 4), img]
        if alt.strip():
            parts.append(make_para(inline(alt), self.styles['caption']))
        else:
            parts.append(Spacer(1, 10))
        return KeepTogether(parts)

    def _blockquote(self, lines, i, out):
        body = []
        while i < len(lines):
            m = _QUOTE_RE.match(lines[i])
            if m:
                body.append(m.group(1))
            elif lines[i].strip() and body and body[-1].strip():
                body.append(lines[i])  # Lazy continuation.
            else:
                break
            i += 1
        inner = self.render(body, in_quote=True)
        if not inner:
            return i
        rows = [[f] for f in inner]
        table = Table(rows, colWidths=[FRAME_W - 6], hAlign='LEFT')
        table.setStyle(TableStyle([
            ('LINEBEFORE', (0, 0), (0, -1), 2.5, ACCENT),
            ('LEFTPADDING', (0, 0), (-1, -1), 12),
            ('RIGHTPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 1),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 1),
            ('TOPPADDING', (0, 0), (-1, 0), 4),
            ('BOTTOMPADDING', (0, -1), (-1, -1), 2),
        ]))
        out.extend([table, Spacer(1, 8)])
        return i

    def _list(self, lines, i, out, in_quote):
        """Parses a (possibly nested) list starting at ``lines[i]``."""
        items = []  # [indent, marker, text]
        n = len(lines)
        while i < n:
            line = lines[i]
            m = _LIST_RE.match(line)
            if m and not _HR_RE.match(line):
                indent = len(m.group(1).expandtabs(4))
                items.append([indent, m.group(2), m.group(3).strip()])
                i += 1
                continue
            if not line.strip():
                # Continue only if the next non-blank line belongs here.
                j = i + 1
                while j < n and not lines[j].strip():
                    j += 1
                if j < n and (_LIST_RE.match(lines[j])
                              or lines[j].startswith((' ', '\t'))):
                    i = j
                    continue
                break
            if items and (line.startswith((' ', '\t'))
                          or not self._starts_block(lines, i)):
                items[-1][2] += ' ' + line.strip()
                i += 1
                continue
            break

        indents = []
        counters = {}
        base = self.styles['quote' if in_quote else 'body']
        for indent, marker, text in items:
            while indents and indent < indents[-1]:
                indents.pop()
            if not indents or indent > indents[-1] + 1:
                if not indents or indent > indents[-1]:
                    indents.append(indent)
            level = len(indents) - 1
            for deeper in [k for k in counters if k > level]:
                del counters[deeper]
            if marker[0].isdigit():
                start = int(marker[:-1])
                count = counters.get(level, start - 1) + 1
                counters[level] = count
                bullet = f'{count}.'
            else:
                counters.pop(level, None)
                bullet = BULLETS[min(level, len(BULLETS) - 1)]
            task = re.match(r'\[([ xX])\]\s+', text)
            if task:
                box = '☐' if task.group(1) == ' ' else '☑'
                text = text[task.end():]
                bullet = box
            left = 16 + 16 * level
            style = ParagraphStyle(
                f'li{level}', parent=base, leftIndent=left,
                bulletIndent=left - 12, bulletFontName=SANS,
                spaceAfter=2.5, bulletFontSize=base.fontSize)
            out.append(make_para(inline(text), style, bulletText=bullet))
        out.append(Spacer(1, 4))
        return i

    def _table(self, lines, i, out):
        header = _split_row(lines[i])
        aligns = []
        for spec in _split_row(lines[i + 1]):
            left, right = spec.startswith(':'), spec.endswith(':')
            aligns.append(TA_CENTER if left and right else
                          TA_RIGHT if right else TA_LEFT)
        rows = []
        i += 2
        while i < len(lines) and lines[i].strip() and '|' in lines[i]:
            rows.append(_split_row(lines[i]))
            i += 1
        ncols = max([len(header)] + [len(r) for r in rows])
        grid = [header] + rows
        grid = [r + [''] * (ncols - len(r)) for r in grid]
        aligns += [TA_LEFT] * (ncols - len(aligns))
        out.append(self._build_table(grid, aligns))
        out.append(Spacer(1, 10))
        return i

    def _build_table(self, grid, aligns):
        """Builds a sized and styled ``Table`` from cell strings."""
        ncols = len(grid[0])
        size, widths = _fit_columns(grid, ncols)
        body = self.styles['body']
        cells = []
        for r, row in enumerate(grid):
            out_row = []
            for c, text in enumerate(row):
                style = ParagraphStyle(
                    f'cell{r > 0}{aligns[c]}', parent=body, fontSize=size,
                    leading=size * 1.3, spaceAfter=0,
                    fontName=f'{SANS}-Bold' if r == 0 else SANS,
                    alignment=aligns[c])
                out_row.append(make_para(inline(text), style))
            cells.append(out_row)
        table = Table(cells, colWidths=widths, repeatRows=1, hAlign='LEFT')
        cmds = [
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
            ('LEFTPADDING', (0, 0), (-1, -1), CELL_PAD),
            ('RIGHTPADDING', (0, 0), (-1, -1), CELL_PAD),
            ('TOPPADDING', (0, 0), (-1, -1), 3),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 3.5),
            ('BACKGROUND', (0, 0), (-1, 0), HEAD_BG),
            ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, ZEBRA]),
            ('INNERGRID', (0, 0), (-1, -1), 0.3, RULE),
            ('BOX', (0, 0), (-1, -1), 0.5, RULE),
            ('LINEBELOW', (0, 0), (-1, 0), 0.8, MUTED),
        ]
        table.setStyle(TableStyle(cmds))
        return table


def _image_dpi(path):
    """Returns the horizontal DPI stored in an image, defaulting to 96."""
    try:
        # pylint: disable-next=import-outside-toplevel
        from PIL import Image as PILImage
        with PILImage.open(path) as im:
            dpi = im.info.get('dpi', (96, 96))[0]
        return float(dpi) if dpi and dpi > 10 else 96.0
    except Exception:  # pylint: disable=broad-except
        return 96.0


def _fit_columns(grid, ncols):
    """Chooses a table font size and column widths that fit the frame.

    Args:
        grid: List of rows (first is the header) of cell strings.
        ncols: Number of columns.

    Returns:
        Tuple ``(font_size, column_widths)``.
    """
    pad = 2 * CELL_PAD + 2
    max_min = 1.3 * inch

    def measure(size):
        nat, mins = [0.0] * ncols, [0.0] * ncols
        for r, row in enumerate(grid):
            font = f'{SANS}-Bold' if r == 0 else SANS
            for c, text in enumerate(row):
                txt = plain_text(text)
                w = pdfmetrics.stringWidth(txt, font, size) + pad
                words = txt.split() or ['']
                mw = max(pdfmetrics.stringWidth(word, font, size)
                         for word in words) + pad
                nat[c] = max(nat[c], w)
                mins[c] = max(mins[c], min(mw, max_min))
        return nat, mins

    # Prefer a moderately smaller font with no wrapping at all.
    for size in [s for s in TABLE_SIZES if s >= MIN_NOWRAP_SIZE]:
        nat, _ = measure(size)
        if sum(nat) <= FRAME_W:
            return size, nat
    # Otherwise wrap, shrinking only until the longest words fit.
    chosen = None
    for size in TABLE_SIZES:
        nat, mins = measure(size)
        if sum(nat) <= FRAME_W:
            return size, nat
        if sum(mins) <= 0.75 * FRAME_W:
            chosen = (size, nat, mins)
            break
    if chosen is None:
        size = TABLE_SIZES[-1]
        chosen = (size,) + measure(size)
    size, nat, mins = chosen
    if sum(mins) > FRAME_W:
        scale = FRAME_W / sum(mins)
        return size, [m * scale for m in mins]
    slack = FRAME_W - sum(mins)
    extra = [nv - mv for nv, mv in zip(nat, mins)]
    total = sum(extra) or 1.0
    return size, [mv + slack * e / total for mv, e in zip(mins, extra)]


# ---------------------------------------------------------------------------
# Document assembly.
# ---------------------------------------------------------------------------


class ReportDoc(BaseDocTemplate):
    """Doc template adding outline entries, TOC notifications and footer."""

    def __init__(self, filename, title, **kwargs):
        super().__init__(filename, pagesize=letter, leftMargin=MARGIN,
                         rightMargin=MARGIN, topMargin=MARGIN,
                         bottomMargin=MARGIN, title=title, **kwargs)
        self.doc_title = title
        frame = Frame(MARGIN, MARGIN, FRAME_W, FRAME_H, id='body',
                      leftPadding=0, rightPadding=0, topPadding=0,
                      bottomPadding=0)
        self.addPageTemplates(
            [PageTemplate(id='page', frames=[frame], onPage=self._footer)])
        self._last_level = -1

    def beforeDocument(self):
        """Resets outline nesting state at the start of every pass."""
        self._last_level = -1

    def _footer(self, canv, doc):
        canv.saveState()
        y = MARGIN * 0.55
        canv.setStrokeColor(RULE)
        canv.setLineWidth(0.5)
        canv.line(MARGIN, y + 10, PAGE_W - MARGIN, y + 10)
        canv.setFont(SANS, 7.5)
        canv.setFillColor(MUTED)
        title = self.doc_title
        max_w = FRAME_W - 60
        while title and pdfmetrics.stringWidth(title, SANS, 7.5) > max_w:
            title = title[:-2] + '…'
        canv.drawString(MARGIN, y, title)
        canv.drawRightString(PAGE_W - MARGIN, y, str(doc.page))
        canv.restoreState()

    def afterFlowable(self, flowable):
        """Registers outline/TOC entries for heading paragraphs."""
        info = getattr(flowable, 'md_heading', None)
        if not info:
            return
        level, text, key = info
        outline_level = min(level - 1, self._last_level + 1)
        self._last_level = outline_level
        self.canv.addOutlineEntry(text, key, level=outline_level,
                                  closed=outline_level >= 1)
        if level == 2:
            self.notify('TOCEntry', (0, text, self.page, key))


def build_pdf(md_path, pdf_path, title=None):
    """Converts a Markdown file to a PDF.

    Args:
        md_path: Path to the input Markdown file.
        pdf_path: Path of the PDF to write.
        title: Optional document title for the footer and metadata;
            defaults to the first H1 or the file stem.
    """
    md_path = Path(md_path)
    register_fonts()
    styles = make_styles()
    source = md_path.read_text(encoding='utf-8')
    lines = source.replace('\r\n', '\n').replace('\r', '\n').split('\n')
    renderer = Renderer(styles, md_path.parent)
    story = renderer.render(lines)

    h1 = [h for h in renderer.headings if h[0] == 1]
    h2 = [h for h in renderer.headings if h[0] == 2]
    if title is None:
        title = h1[0][1] if h1 else md_path.stem

    if len(h2) >= 4:
        toc = TableOfContents(dotsMinLevel=0)
        toc.levelStyles = [styles['toc']]
        block = [make_para('CONTENTS', styles['toc_title']), toc,
                 HRFlowable(width='100%', thickness=0.5, color=RULE,
                            spaceBefore=8, spaceAfter=6)]
        pos = 0
        for idx, flow in enumerate(story):
            if getattr(flow, 'md_heading', (0,))[0] == 1:
                pos = idx + 1
                break
        story[pos:pos] = block

    if not story:
        story = [Spacer(1, 1)]
    doc = ReportDoc(str(pdf_path), title)
    doc.multiBuild(story)


def main(argv=None):
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('input', help='Markdown file')
    parser.add_argument('output', help='PDF file to write')
    parser.add_argument('--title', default=None,
                        help='Document title (footer and PDF metadata)')
    args = parser.parse_args(argv)
    if not Path(args.input).is_file():
        parser.error(f'input not found: {args.input}')
    build_pdf(args.input, args.output, title=args.title)
    print(f'Wrote {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
