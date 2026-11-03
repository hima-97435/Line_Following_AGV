"""ReportLab layout implementing the AISR report format rules.

A4 portrait; margins L 3.0 cm, R/T/B 2.5 cm; single-line border on every page
(including the title page); Times New Roman 12 pt body with 1.5 line spacing,
justified; 14 pt bold section and 12 pt bold subsection headings; page numbers
bottom-centre, roman on front pages and arabic from Section 1; automatically
generated Index, List of Figures and List of Tables.
"""
import os

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, Frame, Image, KeepTogether, PageTemplate, Paragraph,
                                Preformatted, Spacer, Table, TableStyle)
from reportlab.platypus.tableofcontents import TableOfContents


# ------------------------------------------------------------------ fonts
def register_fonts():
    candidates = ["/mnt/c/Windows/Fonts", "C:/Windows/Fonts", "/usr/share/fonts/truetype/msttcorefonts"]
    for d in candidates:
        if os.path.exists(os.path.join(d, "times.ttf")):
            pdfmetrics.registerFont(TTFont("TNR", os.path.join(d, "times.ttf")))
            pdfmetrics.registerFont(TTFont("TNR-B", os.path.join(d, "timesbd.ttf")))
            pdfmetrics.registerFont(TTFont("TNR-I", os.path.join(d, "timesi.ttf")))
            pdfmetrics.registerFont(TTFont("TNR-BI", os.path.join(d, "timesbi.ttf")))
            pdfmetrics.registerFontFamily("TNR", normal="TNR", bold="TNR-B", italic="TNR-I", boldItalic="TNR-BI")
            mono = os.path.join(d, "consola.ttf")
            if os.path.exists(mono):
                pdfmetrics.registerFont(TTFont("Mono", mono))
                return "TNR", "Mono"
            return "TNR", "Courier"
    return "Times-Roman", "Courier"   # base-14 fallback (metric-compatible with Times New Roman)


FONT, MONO = register_fonts()
BOLD = "TNR-B" if FONT == "TNR" else "Times-Bold"

ST = {
    "body": ParagraphStyle("body", fontName=FONT, fontSize=12, leading=18, alignment=TA_JUSTIFY, spaceAfter=3),
    "bullet": ParagraphStyle("bullet", fontName=FONT, fontSize=12, leading=18, alignment=TA_JUSTIFY,
                             leftIndent=18, bulletIndent=6, spaceAfter=0),
    "H1": ParagraphStyle("H1", fontName=BOLD, fontSize=14, leading=20, spaceBefore=6, spaceAfter=8,
                         keepWithNext=1),
    "H1u": ParagraphStyle("H1u", fontName=BOLD, fontSize=14, leading=20, spaceBefore=6, spaceAfter=8,
                          keepWithNext=1, alignment=TA_CENTER),
    "H2": ParagraphStyle("H2", fontName=BOLD, fontSize=12, leading=18, spaceBefore=5, spaceAfter=2,
                         keepWithNext=1),
    "FigCap": ParagraphStyle("FigCap", fontName=FONT, fontSize=11, leading=14, alignment=TA_CENTER,
                             spaceBefore=3, spaceAfter=6),
    "TabCap": ParagraphStyle("TabCap", fontName=FONT, fontSize=11, leading=14, alignment=TA_CENTER,
                             spaceBefore=6, spaceAfter=4, keepWithNext=1),
    "cell": ParagraphStyle("cell", fontName=FONT, fontSize=10, leading=12.5, alignment=TA_LEFT),
    "cellc": ParagraphStyle("cellc", fontName=FONT, fontSize=10, leading=12.5, alignment=TA_CENTER),
    "cellh": ParagraphStyle("cellh", fontName=BOLD, fontSize=10, leading=12.5, alignment=TA_CENTER),
    "eq": ParagraphStyle("eq", fontName=FONT, fontSize=12, leading=18, alignment=TA_CENTER, spaceBefore=2,
                         spaceAfter=6),
    "code": ParagraphStyle("code", fontName=MONO, fontSize=8.2, leading=10, leftIndent=8, spaceBefore=4,
                           spaceAfter=8, backColor=colors.HexColor("#f4f4f4"), borderPadding=4),
    "small": ParagraphStyle("small", fontName=FONT, fontSize=10, leading=13, alignment=TA_JUSTIFY),
    "title": ParagraphStyle("title", fontName=BOLD, fontSize=19, leading=25, alignment=TA_CENTER),
    "center": ParagraphStyle("center", fontName=FONT, fontSize=12, leading=18, alignment=TA_CENTER),
    "centerb": ParagraphStyle("centerb", fontName=BOLD, fontSize=12, leading=18, alignment=TA_CENTER),
}


def roman(n):
    vals = [(10, "x"), (9, "ix"), (5, "v"), (4, "iv"), (1, "i")]
    out = ""
    for v, s in vals:
        while n >= v:
            out += s
            n -= v
    return out


# TableOfContents needs integer page numbers: front-matter pages are passed as
# negative integers and rendered as roman numerals here.
import reportlab.platypus.tableofcontents as _toc  # noqa: E402
_orig_draw_pages = _toc.drawPageNumbers


def _draw_pages(canvas, style, pages, *args, **kw):
    pages = [(roman(-int(p)) if int(p) < 0 else p, k) for p, k in pages]
    return _orig_draw_pages(canvas, style, pages, *args, **kw)


_toc.drawPageNumbers = _draw_pages


class ListOf(TableOfContents):
    """A TableOfContents that listens to a custom notification kind (figures/tables)."""

    def __init__(self, kind, **kw):
        super().__init__(**kw)
        self.kind = kind

    def notify(self, kind, stuff):
        if kind == self.kind:
            self.addEntry(*stuff)


def make_toc(kind=None):
    lvl = [ParagraphStyle("toc0", fontName=FONT, fontSize=12, leading=14.5, leftIndent=0, firstLineIndent=0),
           ParagraphStyle("toc1", fontName=FONT, fontSize=11.5, leading=13.5, leftIndent=22, firstLineIndent=0)]
    t = TableOfContents() if kind is None else ListOf(kind)
    t.levelStyles = lvl
    t.dotsMinLevel = 0
    return t


class ReportDoc(BaseDocTemplate):
    BORDER = (2.0 * cm, 1.5 * cm, 1.5 * cm, 1.5 * cm)   # left, right, top, bottom inset of the page border

    def __init__(self, filename, **kw):
        super().__init__(filename, pagesize=A4, leftMargin=3.0 * cm, rightMargin=2.5 * cm, topMargin=2.5 * cm,
                         bottomMargin=2.5 * cm, **kw)
        fr = Frame(self.leftMargin, self.bottomMargin, self.width, self.height, id="body",
                   leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)
        self.addPageTemplates([
            PageTemplate("title", [fr], onPage=self._on_title),
            PageTemplate("front", [fr], onPage=self._on_front),
            PageTemplate("main", [fr], onPage=self._on_main),
        ])
        self.main_start = None
        self._key = 0

    def handle_documentBegin(self):
        super().handle_documentBegin()
        self.main_start = None
        self._key = 0

    # ----------------------------------------------------------- decorations
    def _border(self, canv):
        W, H = A4
        l, r, t, b = self.BORDER
        canv.saveState()
        canv.setLineWidth(0.8)
        canv.rect(l, b, W - l - r, H - t - b)
        canv.restoreState()

    def _number(self, canv, label):
        canv.saveState()
        canv.setFont(FONT, 11)
        canv.drawCentredString(self.leftMargin + self.width / 2, 1.85 * cm, label)
        canv.restoreState()

    def _on_title(self, canv, doc):
        self._border(canv)

    def _on_front(self, canv, doc):
        self._border(canv)
        self._number(canv, roman(doc.page - 1))

    def _on_main(self, canv, doc):
        if self.main_start is None:
            self.main_start = doc.page
        self._border(canv)
        self._number(canv, str(doc.page - self.main_start + 1))

    def label(self):
        """Displayed page number: positive int (arabic) or negative int (roman)."""
        if self.pageTemplate.id == "main" and self.main_start is not None:
            return self.page - self.main_start + 1
        return -(self.page - 1)

    # --------------------------------------------------------------- indexing
    def afterFlowable(self, f):
        if not isinstance(f, Paragraph):
            return
        name = f.style.name
        text = f.getPlainText()
        if name in ("H1", "H1u", "H2"):
            if getattr(f, "_no_toc", False):
                return
            level = 1 if name == "H2" else 0
            self._key += 1
            key = f"k{self._key}"
            self.canv.bookmarkPage(key)
            self.canv.addOutlineEntry(text, key, level=level, closed=level > 0)
            self.notify("TOCEntry", (level, text, self.label(), key))
        elif name == "FigCap":
            self.notify("FigEntry", (0, text, self.label()))
        elif name == "TabCap":
            self.notify("TabEntry", (0, text, self.label()))


# --------------------------------------------------------------- builders
def P(text, style="body"):
    return Paragraph(text, ST[style])


def H1(text, toc=True):
    p = Paragraph(text, ST["H1"])
    p._no_toc = not toc
    return p


def H1u(text, toc=True):
    p = Paragraph(text, ST["H1u"])
    p._no_toc = not toc
    return p


def H2(text):
    return Paragraph(text, ST["H2"])


def bullets(items, style="bullet"):
    return [Paragraph(it, ST[style], bulletText="\u2022") for it in items]


def numbered(items):
    return [Paragraph(it, ST["bullet"], bulletText=f"{i}.") for i, it in enumerate(items, 1)]


def figure(path, caption, width_frac=0.95, max_h=10.5 * cm, frame_w=15.5 * cm):
    from PIL import Image as PILImage
    with PILImage.open(path) as im:
        w, h = im.size
    W = frame_w * width_frac
    H = W * h / w
    if H > max_h:
        H = max_h
        W = H * w / h
    return KeepTogether([Image(path, width=W, height=H), Paragraph(caption, ST["FigCap"])])


def table(caption, rows, col_widths, header_rows=1, align_center_from=1, font_size=None, zebra=True):
    data = []
    for i, row in enumerate(rows):
        cells = []
        for j, c in enumerate(row):
            if isinstance(c, str):
                st = "cellh" if i < header_rows else ("cellc" if j >= align_center_from else "cell")
                cells.append(Paragraph(c, ST[st]))
            else:
                cells.append(c)
        data.append(cells)
    t = Table(data, colWidths=col_widths, repeatRows=header_rows)
    style = [("GRID", (0, 0), (-1, -1), 0.5, colors.black),
             ("BACKGROUND", (0, 0), (-1, header_rows - 1), colors.HexColor("#e6e6e6")),
             ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
             ("TOPPADDING", (0, 0), (-1, -1), 2.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5),
             ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4)]
    if zebra:
        for r in range(header_rows, len(rows)):
            if (r - header_rows) % 2 == 1:
                style.append(("BACKGROUND", (0, r), (-1, r), colors.HexColor("#f7f7f7")))
    t.setStyle(TableStyle(style))
    return [Paragraph(caption, ST["TabCap"]), t, Spacer(1, 8)]


def code(text):
    return Preformatted(text, ST["code"])


__all__ = ["ReportDoc", "P", "H1", "H1u", "H2", "bullets", "numbered", "figure", "table", "code", "make_toc",
           "ST", "FONT", "BOLD", "Spacer", "KeepTogether", "cm", "colors", "Table", "TableStyle", "Paragraph"]
