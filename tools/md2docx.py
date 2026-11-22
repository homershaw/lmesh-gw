"""Turn the project's Markdown documents into Word files for printing and sharing.

    pip install python-docx pillow
    python tools/md2docx.py            # writes docs/*.docx

Handles what these documents use: headings, paragraphs, nested bullets,
numbered lists, tables, code blocks, bold, inline code and links. The two
Mermaid diagrams in DESIGN.md are redrawn as pictures (Word cannot render
Mermaid). The Markdown files stay the source; rerun this after editing them.
"""
import io
import os
import re
import shutil
import sys
import zipfile

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Arial and Courier New exist on Windows and macOS, and every Linux desktop maps
# them to metric-compatible fonts (Liberation Sans / Mono), so pages break the
# same everywhere. Calibri and Consolas are Windows-only and fell back badly.
BODY_FONT, CODE_FONT = 'Arial', 'Courier New'
INK, MUTED, ACCENT = RGBColor(0x1D, 0x2B, 0x36), RGBColor(0x5B, 0x67, 0x70), RGBColor(0x1F, 0x5F, 0xBF)

DOCS = [
    ('docs/DESIGN.md', 'docs/LMESH-Design-Notes.docx', 'LMESH design notes'),
    ('docs/WORKLOG-2026-11-21.md', 'docs/LMESH-Work-Log-2026-11-21.docx', 'LMESH work log, 21 November 2026'),
    ('node/README.md', 'docs/LMESH-Field-Node-Guide.docx', 'LMESH field node guide'),
    ('README.md', 'docs/LMESH-Gateway-Guide.docx', 'LMESH gateway guide'),
]


# ---------------------------------------------------------------- diagrams
def _font(size, bold=False):
    from PIL import ImageFont
    for name in (('arialbd.ttf' if bold else 'arial.ttf'), 'DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


class Canvas:
    """Tiny box-and-line drawing helper; coordinates in pixels at 2x for sharp print."""

    def __init__(self, w, h):
        from PIL import Image, ImageDraw
        self.img = Image.new('RGB', (w, h), 'white')
        self.d = ImageDraw.Draw(self.img)
        self.f, self.fb, self.fs = _font(26), _font(26, True), _font(21)

    def box(self, x, y, w, h, title, lines=(), fill='#eef5ff'):
        self.d.rounded_rectangle((x, y, x + w, y + h), 14, fill=fill, outline='#1d2b36', width=3)
        rows = [(title, self.fb)] + [(t, self.fs) for t in lines]
        total = sum(34 if f is self.fb else 28 for _, f in rows)
        cy = y + (h - total) / 2
        for t, f in rows:
            tw = self.d.textlength(t, font=f)
            self.d.text((x + (w - tw) / 2, cy), t, font=f, fill='#1d2b36')
            cy += 34 if f is self.fb else 28
        return (x, y, w, h)

    def link(self, a, b, label='', dashed=False, side='h'):
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        if side == 'h':
            p, q = (ax + aw, ay + ah / 2), (bx, by + bh / 2)
        else:
            p, q = (ax + aw / 2, ay + ah), (bx + bw / 2, by)
        if dashed:
            n = 14
            for i in range(0, n, 2):
                self.d.line((p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n,
                             p[0] + (q[0] - p[0]) * (i + 1) / n, p[1] + (q[1] - p[1]) * (i + 1) / n), fill='#1f5fbf', width=5)
        else:
            self.d.line((p, q), fill='#1d2b36', width=3)
        if label:
            tw = self.d.textlength(label, font=self.fs)
            mx, my = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
            if side == 'h':
                self.d.text((mx - tw / 2, my - 30), label, font=self.fs, fill='#5b6770')
            else:
                self.d.text((mx + 10, my - 12), label, font=self.fs, fill='#5b6770')

    def png(self):
        b = io.BytesIO()
        self.img.save(b, 'PNG')
        b.seek(0)
        return b


def diagram_system():
    c = Canvas(2000, 700)
    rtu = c.box(30, 90, 300, 150, 'RTU / meter', ['Modbus RTU slave'], '#f7f9fb')
    node = c.box(470, 90, 320, 150, 'Field node', ['ESP32-C6 + E52', 'site ID = LSD'])
    relay = c.box(900, 330, 300, 130, 'Relay', ['any node or bare E52'], '#f7f9fb')
    gw = c.box(1290, 90, 340, 150, 'Gateway', ['E52 router 0x0001', 'ESP32-S31'])
    scada = c.box(1700, 20, 280, 110, 'SCADA / HMI', ['Modbus TCP :502'], '#f7f9fb')
    broker = c.box(1700, 170, 280, 110, 'MQTT broker', ['dashboards, cloud'], '#f7f9fb')
    tech = c.box(1700, 320, 280, 110, 'Technician', ['Wi-Fi, web page'], '#f7f9fb')
    c.link(rtu, node, 'RS-485')
    c.link(node, gw, 'LoRa mesh 915 MHz', dashed=True)
    c.d.line((node[0] + 160, node[1] + 150, relay[0], relay[1] + 65), fill='#1f5fbf', width=5)
    c.d.line((relay[0] + 300, relay[1] + 65, gw[0] + 170, gw[1] + 150), fill='#1f5fbf', width=5)
    c.d.text((930, 480), 'multi-hop through other nodes', font=c.fs, fill='#5b6770')
    for dst, lab in ((scada, 'Modbus TCP :502'), (broker, 'MQTT'), (tech, 'Wi-Fi')):
        c.d.line((gw[0] + 340, gw[1] + 75, dst[0], dst[1] + 55), fill='#1d2b36', width=3)
    c.d.text((30, 620), 'One field site per mast. The radios form the mesh on their own; LMESH adds ports, retries and acknowledgements.',
             font=c.fs, fill='#5b6770')
    return c.png()


def diagram_gateway():
    c = Canvas(2000, 820)
    top = [c.box(30 + i * 400, 30, 360, 140, t, l, '#f7f9fb') for i, (t, l) in enumerate([
        ('Web UI :80', ['login · worker tasks']), ('Modbus TCP :502', ['unit → node map', 'retransmit']),
        ('MQTT bridge', ['site-ID topics', 'acked downlinks']), ('UDP bridge :5252', [])])]
    lm = c.box(30, 270, 1560, 120, 'LMESH layer (shared/lmesh.c)', ['fragments · node table · ANNOUNCE · ping'])
    drv = c.box(30, 480, 1560, 120, 'E52 driver (shared/e52.c)', ['AT commands · framed receive · transmit pacing · module check'])
    e52 = c.box(560, 690, 500, 110, 'E52-900NW22S, router 0x0001', [], '#f7f9fb')
    wifi = c.box(1660, 270, 310, 120, 'Wi-Fi', ['AP always', 'STA if set'], '#f7f9fb')
    for b in top:
        c.link(b, (b[0], lm[1], b[2], lm[3]), side='v')
    c.link(lm, drv, side='v')
    c.link((e52[0], drv[1], e52[2], drv[3]), e52, 'USB (CH340) today, UART on the daughter board', side='v')
    c.d.text((1660, 410), 'AP LMESH-GW-xxxx', font=c.fs, fill='#5b6770')
    c.d.text((1660, 440), 'and the site LAN', font=c.fs, fill='#5b6770')
    return c.png()


DIAGRAMS = {'docs/DESIGN.md': [(diagram_system, 'System overview'), (diagram_gateway, 'Gateway firmware (main/)')]}


# ---------------------------------------------------------------- Word helpers
def shade(cell_or_par, hex_fill):
    pr = cell_or_par._tc.get_or_add_tcPr() if hasattr(cell_or_par, '_tc') else cell_or_par._p.get_or_add_pPr()
    shd = OxmlElement('w:shd')
    shd.set(qn('w:val'), 'clear')
    shd.set(qn('w:color'), 'auto')
    shd.set(qn('w:fill'), hex_fill)
    pr.append(shd)


def page_number_field(par):
    for kind, text in (('begin', None), (None, 'PAGE'), ('end', None)):
        run = par.add_run()
        if kind:
            el = OxmlElement('w:fldChar')
            el.set(qn('w:fldCharType'), kind)
        else:
            el = OxmlElement('w:instrText')
            el.set(qn('xml:space'), 'preserve')
            el.text = text
        run._r.append(el)
        run.font.size = Pt(9)
        run.font.color.rgb = MUTED


INLINE = re.compile(r'(\*\*.+?\*\*|`[^`]+`|\[[^\]]+\]\([^)]+\))')


def add_inline(par, text, size=None, bold=False):
    """Bold, inline code and links. A link to a file in the repo keeps its text; a web link shows its address."""
    for part in INLINE.split(text):
        if not part:
            continue
        if part.startswith('**') and part.endswith('**'):
            add_inline(par, part[2:-2], size, True)
            continue
        if part.startswith('`') and part.endswith('`'):
            r = par.add_run(part[1:-1])
            r.font.name = CODE_FONT
            r._r.get_or_add_rPr().get_or_add_rFonts().set(qn('w:eastAsia'), CODE_FONT)
            r.font.size = Pt((size or 11) - 1)
            r.bold = bold
            continue
        m = re.fullmatch(r'\[([^\]]+)\]\(([^)]+)\)', part)
        if m:
            r = par.add_run(m.group(1))
            r.bold = bold
            if m.group(2).startswith('http'):
                r.font.color.rgb = ACCENT
                r2 = par.add_run(' (' + m.group(2) + ')')
                r2.font.size = Pt(9)
                r2.font.color.rgb = MUTED
            if size:
                r.font.size = Pt(size)
            continue
        r = par.add_run(part.replace('<br/>', ' '))
        r.bold = bold
        if size:
            r.font.size = Pt(size)


def split_row(line):
    cells, cur, in_code = [], '', False
    for ch in line.strip().strip('|'):
        if ch == '`':
            in_code = not in_code
        if ch == '|' and not in_code:
            cells.append(cur.strip())
            cur = ''
        else:
            cur += ch
    cells.append(cur.strip())
    return cells


def add_table(doc, rows):
    head, body = rows[0], rows[2:]
    ncol = len(head)
    t = doc.add_table(rows=1, cols=ncol)
    t.style = 'Table Grid'
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    for i, text in enumerate(head):
        c = t.rows[0].cells[i]
        shade(c, 'DCE6F2')
        add_inline(c.paragraphs[0], text, 10, True)
    for row in body:
        cells = t.add_row().cells
        for i in range(ncol):
            add_inline(cells[i].paragraphs[0], row[i] if i < len(row) else '', 10)
    # Column widths follow the content: short label columns stay narrow and the
    # text columns get the room. Set on every cell, which is what Word, ONLYOFFICE
    # and LibreOffice all honour.
    plain = lambda s: re.sub(r'[*`]|\]\([^)]*\)|\[', '', s)
    want = []
    for i in range(ncol):
        longest = max(len(plain(r[i])) if i < len(r) else 0 for r in [head] + body)
        want.append(min(max(longest, 6), 60))
    total_in, floor_in = 6.5, 0.8
    widths = [max(floor_in, total_in * w / sum(want)) for w in want]
    scale = total_in / sum(widths)
    widths = [w * scale for w in widths]
    t.autofit = False
    for row in t.rows:
        for i, c in enumerate(row.cells):
            c.width = Inches(widths[i])
    # repeat the header row when the table breaks across pages
    trpr = t.rows[0]._tr.get_or_add_trPr()
    el = OxmlElement('w:tblHeader')
    el.set(qn('w:val'), 'true')
    trpr.append(el)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)


def add_code(doc, lines):
    for i, ln in enumerate(lines):
        p = doc.add_paragraph()
        p.paragraph_format.space_before = Pt(0)
        p.paragraph_format.space_after = Pt(0)
        p.paragraph_format.left_indent = Inches(0.2)
        p.paragraph_format.keep_together = True
        shade(p, 'F2F4F6')
        r = p.add_run(ln if ln else ' ')
        r.font.name = CODE_FONT
        r._r.get_or_add_rPr().get_or_add_rFonts().set(qn('w:eastAsia'), CODE_FONT)
        r.font.size = Pt(9)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)


def new_document(title):
    doc = Document()
    sec = doc.sections[0]
    sec.page_width, sec.page_height = Inches(8.5), Inches(11)          # US Letter
    sec.left_margin = sec.right_margin = Inches(1)
    sec.top_margin = sec.bottom_margin = Inches(0.9)
    st = doc.styles['Normal']
    st.font.name, st.font.size, st.font.color.rgb = BODY_FONT, Pt(10.5), INK
    st.paragraph_format.space_after = Pt(6)
    for name, size, before, color in (('Title', 24, 0, INK), ('Heading 1', 17, 18, ACCENT), ('Heading 2', 13.5, 14, INK), ('Heading 3', 12, 10, INK)):
        h = doc.styles[name]
        h.font.name, h.font.size, h.font.bold, h.font.color.rgb = BODY_FONT, Pt(size), True, color
        rf = h.element.get_or_add_rPr().get_or_add_rFonts()
        for a in ('w:asciiTheme', 'w:hAnsiTheme', 'w:eastAsiaTheme', 'w:cstheme'):      # theme fonts would override ours
            if rf.get(qn(a)) is not None:
                del rf.attrib[qn(a)]
        rf.set(qn('w:ascii'), BODY_FONT)
        rf.set(qn('w:hAnsi'), BODY_FONT)
        h.paragraph_format.space_before, h.paragraph_format.space_after = Pt(before), Pt(6)
        h.paragraph_format.keep_with_next = True
    hp = sec.header.paragraphs[0]
    r = hp.add_run(title)
    r.font.size, r.font.color.rgb = Pt(9), MUTED
    fp = sec.footer.paragraphs[0]
    fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    page_number_field(fp)
    return doc


def make_portable(path):
    """Remove everything in the file that depends on fonts a machine may not have.

    python-docx starts from a Word template whose defaults point at theme fonts
    (Calibri, Cambria), whose theme lists a font per script (Japanese, Chinese,
    Thai...), and whose bullets are a private-use character in the Symbol font.
    On a machine without those fonts the text falls back unpredictably, the font
    list fills with foreign names and bullets turn into boxes. Rewrite those
    parts to name only BODY_FONT and CODE_FONT and to use a real bullet character.
    """
    def fix_rfonts(m):
        tag = re.sub(r'\s+w:(?:asciiTheme|hAnsiTheme|eastAsiaTheme|cstheme)="[^"]*"', '', m.group(0))
        for attr in ('ascii', 'hAnsi', 'eastAsia', 'cs'):
            if ('w:%s="' % attr) not in tag:
                tag = tag.replace('<w:rFonts', '<w:rFonts w:%s="%s"' % (attr, BODY_FONT), 1)
        return re.sub(r'w:(ascii|hAnsi|eastAsia|cs)="(?!%s")[^"]*"' % re.escape(CODE_FONT), r'w:\1="%s"' % BODY_FONT, tag)

    tmp = path + '.tmp'
    with zipfile.ZipFile(path) as zin, zipfile.ZipFile(tmp, 'w', zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            data = zin.read(item.filename)
            name = item.filename
            if name in ('word/stylesWithEffects.xml', 'docProps/thumbnail.jpeg'):
                continue                                   # legacy duplicate of styles; stock preview picture
            if name == 'word/styles.xml':
                t = data.decode('utf-8')
                t = re.sub(r'<w:rFonts\b[^>]*/>', fix_rfonts, t)
                data = t.encode('utf-8')
            elif name == 'word/theme/theme1.xml':
                t = data.decode('utf-8')
                t = re.sub(r'<a:font script="[^"]*" typeface="[^"]*"/>', '', t)
                t = re.sub(r'<a:latin typeface="[^"]*"', '<a:latin typeface="%s"' % BODY_FONT, t)
                data = t.encode('utf-8')
            elif name == 'word/numbering.xml':
                t = data.decode('utf-8')
                t = t.replace('\uf0b7', '\u2022').replace('\uf0a7', '\u25aa')
                t = re.sub(r'<w:rFonts w:ascii="(?:Symbol|Wingdings)" w:hAnsi="(?:Symbol|Wingdings)"[^>]*/>',
                           '<w:rFonts w:ascii="%s" w:hAnsi="%s" w:cs="%s"/>' % (BODY_FONT, BODY_FONT, BODY_FONT), t)
                data = t.encode('utf-8')
            elif name == 'word/fontTable.xml':
                head = data.decode('utf-8').split('<w:font ', 1)[0]
                fonts = ''.join('<w:font w:name="%s"><w:charset w:val="00"/><w:family w:val="%s"/><w:pitch w:val="%s"/></w:font>' % f
                                for f in ((BODY_FONT, 'swiss', 'variable'), (CODE_FONT, 'modern', 'fixed')))
                data = (head + fonts + '</w:fonts>').encode('utf-8')
            elif name == '[Content_Types].xml':
                t = data.decode('utf-8')
                t = re.sub(r'<Override PartName="/word/stylesWithEffects.xml"[^>]*/>', '', t)
                t = re.sub(r'<Override PartName="/docProps/thumbnail.jpeg"[^>]*/>', '', t)
                data = t.encode('utf-8')
            elif name.endswith('.rels'):
                t = data.decode('utf-8')
                t = re.sub(r'<Relationship [^>]*Target="[^"]*(?:stylesWithEffects\.xml|thumbnail\.jpeg)"[^>]*/>', '', t)
                data = t.encode('utf-8')
            if name.endswith('/'):
                continue
            zout.writestr(item.filename, data)
    shutil.move(tmp, path)


# ---------------------------------------------------------------- converter
def convert(src, dst, title):
    lines = open(os.path.join(ROOT, src), encoding='utf-8').read().replace('\r\n', '\n').split('\n')
    doc = new_document(title)
    diagrams = list(DIAGRAMS.get(src, []))
    i, first_h1 = 0, True
    while i < len(lines):
        ln = lines[i]
        s = ln.strip()
        if not s or s == '---':
            i += 1
            continue
        if s.startswith('```'):
            lang = s[3:].strip()
            block = []
            i += 1
            while i < len(lines) and not lines[i].strip().startswith('```'):
                block.append(lines[i].rstrip())
                i += 1
            i += 1
            if lang == 'mermaid':
                if diagrams:
                    fn, caption = diagrams.pop(0)
                    doc.add_picture(fn(), width=Inches(6.5))
                    doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
                    cp = doc.add_paragraph()
                    cp.alignment = WD_ALIGN_PARAGRAPH.CENTER
                    cr = cp.add_run(caption)
                    cr.italic, cr.font.size, cr.font.color.rgb = True, Pt(9.5), MUTED
                continue
            add_code(doc, block)
            continue
        m = re.match(r'(#{1,4})\s+(.*)', s)
        if m:
            level = len(m.group(1))
            if level == 1 and first_h1:
                p = doc.add_paragraph(style='Title')
                first_h1 = False
            else:
                p = doc.add_paragraph(style='Heading %d' % min(3, max(1, level - 1)))
            add_inline(p, m.group(2))
            i += 1
            continue
        if s.startswith('|') and i + 1 < len(lines) and re.match(r'\s*\|[\s:|-]+\|\s*$', lines[i + 1]):
            rows = []
            while i < len(lines) and lines[i].strip().startswith('|'):
                rows.append(split_row(lines[i]))
                i += 1
            add_table(doc, rows)
            continue
        m = re.match(r'(\s*)([-*]|\d+\.)\s+(.*)', ln)
        if m:
            depth = min(2, len(m.group(1)) // 2)
            numbered = m.group(2)[0].isdigit()
            text = m.group(3)
            # a wrapped list item continues on the following indented lines
            while i + 1 < len(lines) and lines[i + 1].strip() and re.match(r'\s{2,}\S', lines[i + 1]) and not re.match(r'\s*([-*]|\d+\.)\s', lines[i + 1]):
                i += 1
                text += ' ' + lines[i].strip()
            if numbered:
                p = doc.add_paragraph()
                p.paragraph_format.left_indent = Inches(0.3 + 0.25 * depth)
                p.paragraph_format.first_line_indent = Inches(-0.3)
                p.paragraph_format.space_after = Pt(3)
                p.add_run(m.group(2) + '\t')
                p.paragraph_format.tab_stops.add_tab_stop(Inches(0.3 + 0.25 * depth))
            else:
                p = doc.add_paragraph(style='List Bullet' if depth == 0 else 'List Bullet %d' % (depth + 1))
                p.paragraph_format.space_after = Pt(3)
            add_inline(p, text)
            i += 1
            continue
        # ordinary paragraph: join soft-wrapped lines
        text = s
        while i + 1 < len(lines) and lines[i + 1].strip() and not re.match(r'\s*([-*]|\d+\.|#{1,4}|\||```)', lines[i + 1]):
            i += 1
            text += ' ' + lines[i].strip()
        add_inline(doc.add_paragraph(), text)
        i += 1
    out = os.path.join(ROOT, dst)
    doc.save(out)
    make_portable(out)
    return out


if __name__ == '__main__':
    for src, dst, title in DOCS:
        if os.path.exists(os.path.join(ROOT, src)):
            print('wrote', convert(src, dst, title))
