#!/usr/bin/env python3
"""Render CLASP-STD-001_RevA_Working_Draft.md to a clean print-ready PDF via headless Chrome."""

from __future__ import annotations

import html
import re
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MD_PATH = ROOT / "CLASP-STD-001_RevA_Working_Draft.md"
PDF_PATH = ROOT / "CLASP-STD-001_RevA_Working_Draft.pdf"


def inline_md(text: str) -> str:
    """Convert inline markdown (**bold**, *italic*, `code`) safely to HTML."""
    escaped = html.escape(text)
    escaped = re.sub(r"`([^`]+)`", r"<code>\1</code>", escaped)
    escaped = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", escaped)
    escaped = re.sub(r"\*([^*]+)\*", r"<em>\1</em>", escaped)
    return escaped


def md_to_html(md: str) -> str:
    """Minimal, faithful Markdown-to-HTML converter for CLASP-STD-001."""
    lines = md.splitlines()
    out: list[str] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        if not stripped:
            i += 1
            continue

        if stripped == "---":
            out.append("<hr>")
            i += 1
            continue

        m_head = re.match(r"^(#{1,4})\s+(.*)$", stripped)
        if m_head:
            lvl = len(m_head.group(1))
            out.append(f"<h{lvl}>{inline_md(m_head.group(2))}</h{lvl}>")
            i += 1
            continue

        # Table block
        if stripped.startswith("|"):
            table_lines = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                table_lines.append(lines[i].strip())
                i += 1
            if len(table_lines) >= 2:
                headers = [c.strip() for c in table_lines[0].strip("|").split("|")]
                out.append("<table><thead><tr>")
                for h in headers:
                    out.append(f"<th>{inline_md(h)}</th>")
                out.append("</tr></thead><tbody>")
                for row_line in table_lines[2:]:
                    cells = [c.strip() for c in row_line.strip("|").split("|")]
                    out.append("<tr>")
                    for c in cells:
                        out.append(f"<td>{inline_md(c)}</td>")
                    out.append("</tr>")
                out.append("</tbody></table>")
            continue

        # Unordered list
        if stripped.startswith("- "):
            out.append("<ul>")
            while i < len(lines) and lines[i].strip().startswith("- "):
                out.append(f"<li>{inline_md(lines[i].strip()[2:])}</li>")
                i += 1
            out.append("</ul>")
            continue

        # Ordered list
        if re.match(r"^\d+\.\s+", stripped):
            out.append("<ol>")
            while i < len(lines) and re.match(r"^\d+\.\s+", lines[i].strip()):
                item = re.sub(r"^\d+\.\s+", "", lines[i].strip())
                out.append(f"<li>{inline_md(item)}</li>")
                i += 1
            out.append("</ol>")
            continue

        # Paragraph
        para_lines = [stripped]
        i += 1
        while (
            i < len(lines)
            and lines[i].strip()
            and not lines[i].strip().startswith(("#", "|", "- ", "---"))
            and not re.match(r"^\d+\.\s+", lines[i].strip())
        ):
            para_lines.append(lines[i].strip())
            i += 1
        out.append(f"<p>{inline_md(' '.join(para_lines))}</p>")

    body = "\n".join(out)
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>CLASP-STD-001 Revision A Working Draft</title>
<style>
  @page {{
    size: letter;
    margin: 0.65in 0.72in 0.68in 0.72in;
  }}
  body {{
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    font-size: 9.5pt;
    line-height: 1.42;
    color: #1a202c;
  }}
  h1 {{ font-size: 20pt; margin: 0 0 4pt 0; color: #0f172a; letter-spacing: -0.01em; }}
  h2 {{ font-size: 13pt; margin: 16pt 0 6pt 0; padding-bottom: 3pt; border-bottom: 1.5px solid #cbd5e1; color: #0f172a; page-break-after: avoid; }}
  h3 {{ font-size: 10.8pt; margin: 11pt 0 4pt 0; color: #1e293b; page-break-after: avoid; }}
  p {{ margin: 5pt 0; }}
  ul, ol {{ margin: 4pt 0 6pt 0; padding-left: 18pt; }}
  li {{ margin: 2pt 0; }}
  hr {{ border: none; border-top: 1px solid #e2e8f0; margin: 12pt 0; }}
  code {{
    font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    font-size: 8.8pt;
    background: #f1f5f9;
    padding: 1pt 3.5pt;
    border-radius: 3pt;
    color: #0f172a;
  }}
  table {{
    width: 100%;
    border-collapse: collapse;
    margin: 7pt 0 10pt 0;
    font-size: 9pt;
    page-break-inside: avoid;
  }}
  th, td {{
    border: 1px solid #cbd5e1;
    padding: 4.5pt 7pt;
    text-align: left;
    vertical-align: top;
  }}
  th {{ background: #f8fafc; font-weight: 600; color: #0f172a; }}
</style>
</head>
<body>
{body}
</body>
</html>
"""


def main() -> int:
    html_text = md_to_html(MD_PATH.read_text(encoding="utf-8"))
    with tempfile.NamedTemporaryFile("w", suffix=".html", delete=False, encoding="utf-8") as tmp:
        tmp.write(html_text)
        tmp_html = Path(tmp.name)

    try:
        subprocess.run(
            [
                "google-chrome",
                "--headless",
                "--disable-gpu",
                "--no-sandbox",
                f"--print-to-pdf={PDF_PATH}",
                "--no-pdf-header-footer",
                tmp_html.as_uri(),
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    finally:
        tmp_html.unlink(missing_ok=True)

    print(f"Generated {PDF_PATH} ({PDF_PATH.stat().st_size:,} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
