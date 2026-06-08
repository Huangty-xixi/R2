# -*- coding: utf-8 -*-
"""Add emoji (HTML entities) to GROUND_HIGHEST patches in zone2_kfs_flow.html."""
from pathlib import Path

HTML = Path(__file__).resolve().parent / "zone2_kfs_flow.html"

# HTML numeric refs (ASCII in GBK file, renders as emoji in browser)
E_GROUND = "&#127757;"   # earth - ground prep
E_UP = "&#11014;"        # up arrow - p4 / highest
E_BOX = "&#128230;"      # package - get KFS
E_LOOP = "&#128260;"     # arrows - main loop
E_STAR = "&#11088;"      # star - highlight
E_CHART = "&#128202;"    # chart - rel table

REPLACEMENTS = [
    (
        "  \u5730\u9762\u9884\u5907\u53d6\u4ef6\uff1a<b>rel = GROUND_HIGHEST</b>",
        f"  {E_GROUND} \u5730\u9762\u9884\u5907\u53d6\u4ef6\uff1a<b>rel = GROUND_HIGHEST</b>",
    ),
    (
        "\uff0c\u6700\u9ad8\u6863\uff09\uff1b\u6885\u6797\u4e3b\u5faa\u73af",
        f"\uff0c\u6700\u9ad8\u6863 {E_UP}\uff09\uff1b{E_LOOP} \u6885\u6797\u4e3b\u5faa\u73af",
    ),
    (
        '<span class="code">GROUND_HIGHEST</span> \u2192 p4<small>',
        f'<span class="code">GROUND_HIGHEST</span> {E_UP} p4 {E_BOX}<small>',
    ),
    (
        '<span class="code">(1.8,2.6)</span> GetKFS p4<br>',
        f'<span class="code">(1.8,2.6)</span> {E_BOX} GetKFS {E_UP}p4<br>',
    ),
    (
        '<span class="code">(4.2,2.6)</span> GetKFS p4<br>',
        f'<span class="code">(4.2,2.6)</span> {E_BOX} GetKFS {E_UP}p4<br>',
    ),
    (
        '<span class="code">APP_ZONE2_GET_KFS_GROUND_HIGHEST</span> \u2192 \u4e3b\u8f74 p4\uff09',
        f'{E_GROUND}<span class="code">APP_ZONE2_GET_KFS_GROUND_HIGHEST</span> {E_UP}\u2192 \u4e3b\u8f74 p4\uff09',
    ),
    (
        'Process_GetKFS(GROUND_HIGHEST)</span> \u4e3b\u8f74 p4 \u4ece',
        f'{E_BOX}Process_GetKFS(GROUND_HIGHEST)</span> {E_UP}\u4e3b\u8f74 p4 \u4ece',
    ),
    (
        "GetKFS rel \u5206\u6863\uff08\u4e0e app_zone2.h \u4e00\u81f4\uff09",
        f"{E_CHART} GetKFS rel \u5206\u6863\uff08\u4e0e app_zone2.h \u4e00\u81f4\uff09",
    ),
    (
        '<tr><td class="code">GROUND_HIGHEST</td>',
        f'<tr><td class="code">{E_GROUND} GROUND_HIGHEST</td>',
    ),
    (
        '<td>p4\uff08\u6700\u9ad8\u6863\uff09</td></tr>\n'
        '      <tr><td class="code">HIGH_TO_LOW</td>',
        f'<td>{E_UP}p4\uff08\u6700\u9ad8\u6863\uff09</td></tr>\n'
        '      <tr><td class="code">HIGH_TO_LOW</td>',
    ),
    (
        '<span class="code">HIGH/LOW</span> \u6869\u4e0a\u53d6\u77ff\u7c89<small>\u975e GROUND_HIGHEST</small>',
        f'{E_LOOP}<span class="code">HIGH/LOW</span> \u6869\u4e0a\u53d6\u77ff\u7c89<small>\u975e {E_GROUND}GROUND_HIGHEST</small>',
    ),
    (
        "[\u2605 \u4e8c\u533a\u9884\u5907\uff1a\u53ea\u53d6 KFS\u3001GROUND_HIGHEST/p4]",
        f"[{E_STAR} \u4e8c\u533a\u9884\u5907\uff1a\u53ea\u53d6 KFS\u3001{E_GROUND}GROUND_HIGHEST/{E_UP}p4]",
    ),
    (
        "\u53d6\u68692 (p4) -&gt; main_flow",
        f"{E_BOX}\u53d6\u68692 ({E_UP}p4) -&gt; main_flow",
    ),
    (
        "\u53d6\u68691 (p4) -&gt; main_flow",
        f"{E_BOX}\u53d6\u68691 ({E_UP}p4) -&gt; main_flow",
    ),
    (
        "\u53d6\u68693 (p4) -&gt; main_flow",
        f"{E_BOX}\u53d6\u68693 ({E_UP}p4) -&gt; main_flow",
    ),
]


def main():
    text = HTML.read_text(encoding="gbk")
    for old, new in REPLACEMENTS:
        n = text.count(old)
        if n == 0:
            raise SystemExit(f"missing ({n}): {old[:70]!r}")
        if n > 1 and old == "\u53d6\u68692 (p4) -&gt; main_flow":
            text = text.replace(old, new)  # both pile2 lines
        else:
            text = text.replace(old, new, 1)
    HTML.write_text(text, encoding="gbk")
    if HTML.read_bytes().startswith(b"\xef\xbb\xbf"):
        raise SystemExit("BOM detected")
    print("emoji patched OK")


if __name__ == "__main__":
    main()
