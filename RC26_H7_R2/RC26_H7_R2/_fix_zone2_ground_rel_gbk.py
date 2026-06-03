# -*- coding: utf-8 -*-
"""Fix zone2_kfs_flow.html: ground pile2 LOW_TO_HIGH, pile1/3 GROUND_HIGHEST."""
from pathlib import Path

HTML = Path(__file__).resolve().parent / "zone2_kfs_flow.html"

REPLACEMENTS = [
    (
        "  &#127757; \u5730\u9762\u9884\u5907\u53d6\u4ef6\uff1a<b>rel = GROUND_HIGHEST</b>\uff08\u4e3b\u8f74 <span class=\"code\">main_lift_p4</span>\uff0c\u6700\u9ad8\u6863 &#11014;\uff09\uff1b&#128260; \u6885\u6797\u4e3b\u5faa\u73af\u4ecd\u4e3a <span class=\"code\">app_zone2_get_kfs_rel()</span> \u9ad8\u53d6\u4f4e / \u4f4e\u53d6\u9ad8\u3002",
        "  &#127757; \u5730\u9762\u9884\u5907\uff1a<b>\u68691/\u68693</b> <span class=\"code\">GROUND_HIGHEST</span> &#11014;p4\uff1b<b>\u68692</b> <span class=\"code\">LOW_TO_HIGH</span> p3\u3002&#128260; \u6885\u6797\u4e3b\u5faa\u73af\u4ecd\u4e3a <span class=\"code\">app_zone2_get_kfs_rel()</span> \u9ad8\u53d6\u4f4e / \u4f4e\u53d6\u9ad8\u3002",
    ),
    (
        '<span class="code">GROUND_HIGHEST</span> &#11014; p4 &#128230;<small>\u68693\u4e5f\u6709',
        '<span class="code">LOW_TO_HIGH</span> p3 &#128230;<small>\u68693\u4e5f\u6709',
    ),
    (
        '<p class="comic-note">\u9884\u5907\u9636\u6bb5<b>\u53ea\u53d6\u4ef6</b>\uff08&#127757;<span class="code">APP_ZONE2_GET_KFS_GROUND_HIGHEST</span> &#11014;\u2192 \u4e3b\u8f74 p4\uff09\uff1b\u4e0a\u68692 \u7edf\u4e00\u5728',
        '<p class="comic-note">\u9884\u5907\u9636\u6bb5<b>\u53ea\u53d6\u4ef6</b>\uff1a\u68691/\u68693 &#127757; GROUND_HIGHEST &#11014;p4\uff0c\u68692 <span class="code">LOW_TO_HIGH</span> p3\uff1b\u4e0a\u68692 \u7edf\u4e00\u5728',
    ),
    (
        "      <tr><td class=\"code\">&#127757; GROUND_HIGHEST</td><td>Z2_GROUND_PREP \u68691/2/3 \u5730\u9762\u9884\u5907</td><td>&#11014;p4\uff08\u6700\u9ad8\u6863\uff09</td></tr>\n"
        "      <tr><td class=\"code\">HIGH_TO_LOW</td>",
        "      <tr><td class=\"code\">&#127757; GROUND_HIGHEST</td><td>Z2_GROUND_PREP \u68691\u3001\u68693</td><td>&#11014;p4\uff08\u6700\u9ad8\u6863\uff09</td></tr>\n"
        "      <tr><td class=\"code\">LOW_TO_HIGH</td><td>Z2_GROUND_PREP \u68692</td><td>p3\uff08\u4f4e\u53d6\u9ad8\uff09</td></tr>\n"
        "      <tr><td class=\"code\">HIGH_TO_LOW</td>",
    ),
    (
        "      <tr><td class=\"code\">LOW_TO_HIGH</td><td>\u6885\u6797\u4e3b\u5faa\u73af\uff08\u7ad9\u7acb\u6869 &lt; \u90bb\u683c\u6869\uff09</td><td>p3</td></tr>",
        "      <tr><td class=\"code\">LOW_TO_HIGH</td><td>\u6885\u6797\u4e3b\u5faa\u73af\uff08\u7ad9\u7acb\u6869 &lt; \u90bb\u683c\u6869\uff09</td><td>p3</td></tr>",
    ),
    (
        "[&#11088; \u4e8c\u533a\u9884\u5907\uff1a\u53ea\u53d6 KFS\u3001&#127757;GROUND_HIGHEST/&#11014;p4]",
        "[&#11088; \u4e8c\u533a\u9884\u5907\uff1a\u68691/3 p4\u3001\u68692 p3]",
    ),
    (
        "|- \u68692\u6709 -&gt; (3.0,2.6) &#128230;\u53d6\u68692 (&#11014;p4) -&gt; main_flow<br>",
        "|- \u68692\u6709 -&gt; (3.0,2.6) &#128230;\u53d6\u68692 (LOW p3) -&gt; main_flow<br>",
    ),
    (
        "|- \u68692+\u68693\u6709 -&gt; (3.0,2.6) &#128230;\u53d6\u68692 (&#11014;p4) -&gt; main_flow \u4e0a\u6869\u540e\u518d\u53d63<br>",
        "|- \u68692+\u68693\u6709 -&gt; (3.0,2.6) &#128230;\u53d6\u68692 (LOW p3) -&gt; main_flow \u4e0a\u6869\u540e\u518d\u53d63<br>",
    ),
]


def main():
    text = HTML.read_text(encoding="gbk")
    for old, new in REPLACEMENTS:
        if old not in text:
            raise SystemExit(f"missing: {old[:80]!r}")
        text = text.replace(old, new, 1)
    HTML.write_text(text, encoding="gbk")
    print("fixed", HTML.name)


if __name__ == "__main__":
    main()
