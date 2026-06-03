# -*- coding: utf-8 -*-
"""Patch zone2_kfs_flow.html (GBK) for GROUND_HIGHEST / main_lift p4."""
from pathlib import Path

HTML = Path(__file__).resolve().parent / "zone2_kfs_flow.html"

REPLACEMENTS = [
    (
        "\u53d6\u4ef6\u4f18\u5148\u7ea7\uff1a<b>\u68692 &gt; \u68691 &gt; \u68693</b>\uff1b123 \u90fd\u6ca1\u6709 -&gt; \u9884\u5907\u8df3\u8fc7\uff0c\u76f4\u63a5 main_flow\u3002",
        "\u53d6\u4ef6\u4f18\u5148\u7ea7\uff1a<b>\u68692 &gt; \u68691 &gt; \u68693</b>\uff1b123 \u90fd\u6ca1\u6709 -&gt; \u9884\u5907\u8df3\u8fc7\uff0c\u76f4\u63a5 main_flow\u3002<br>\n  \u5730\u9762\u9884\u5907\u53d6\u4ef6\uff1a<b>rel = GROUND_HIGHEST</b>\uff08\u4e3b\u8f74 <span class=\"code\">main_lift_p4</span>\uff0c\u6700\u9ad8\u6863\uff09\uff1b\u6885\u6797\u4e3b\u5faa\u73af\u4ecd\u4e3a <span class=\"code\">app_zone2_get_kfs_rel()</span> \u9ad8\u53d6\u4f4e / \u4f4e\u53d6\u9ad8\u3002",
    ),
    (
        '<div class="comic-bubble"><span class="code">(3.0,2.6)</span> GetKFS<br>-&gt; begin_main_flow()<small>\u68693\u4e5f\u6709\uff1amain_flow \u4e0a\u6869\u540e\u518d\u53d63</small></div>',
        '<div class="comic-bubble"><span class="code">(3.0,2.6)</span> GetKFS<br><span class="code">GROUND_HIGHEST</span> \u2192 p4<small>\u68693\u4e5f\u6709\uff1amain_flow \u4e0a\u6869\u540e\u518d\u53d63</small></div>',
    ),
    (
        '<div class="comic-bubble"><span class="code">(1.8,2.6)</span> \u53d6<br>-&gt; main_flow()<small>1\u548c3\u90fd\u6709\u4e5f\u53ea\u53d61</small></div>',
        '<div class="comic-bubble"><span class="code">(1.8,2.6)</span> GetKFS p4<br>-&gt; main_flow()<small>1\u548c3\u90fd\u6709\u4e5f\u53ea\u53d61</small></div>',
    ),
    (
        '<div class="comic-bubble"><span class="code">(4.2,2.6)</span> \u53d63<br>-&gt; main_flow()</div>',
        '<div class="comic-bubble"><span class="code">(4.2,2.6)</span> GetKFS p4<br>-&gt; main_flow()</div>',
    ),
    (
        '<div class="tl-desc"><span class="code">Process_GetKFS</span> \u4ece\u68691\u5730\u9762\u53d6 KFS\uff0c<b>\u4e0d\u5728\u6b64\u56de\u68692</b></div>',
        '<div class="tl-desc"><span class="code">Process_GetKFS(GROUND_HIGHEST)</span> \u4e3b\u8f74 p4 \u4ece\u68691\u5730\u9762\u53d6 KFS\uff0c<b>\u4e0d\u5728\u6b64\u56de\u68692</b></div>',
    ),
    (
        '<div class="comic-bubble">Process_GetKFS<br>\u6869\u4e0a\u53d6\u77ff\u7c89</div>',
        '<div class="comic-bubble">Process_GetKFS<br><span class="code">HIGH/LOW</span> \u6869\u4e0a\u53d6\u77ff\u7c89<small>\u975e GROUND_HIGHEST</small></div>',
    ),
    (
        "  <span class=\"hl-g\">[\u2605 \u4e8c\u533a\u9884\u5907\uff1a\u53ea\u53d6 KFS]</span><br>\n  |- \u68692\u6709 -&gt; (3.0,2.6) \u53d6\u68692 -&gt; main_flow<br>",
        "  <span class=\"hl-g\">[\u2605 \u4e8c\u533a\u9884\u5907\uff1a\u53ea\u53d6 KFS\u3001GROUND_HIGHEST/p4]</span><br>\n  |- \u68692\u6709 -&gt; (3.0,2.6) \u53d6\u68692 (p4) -&gt; main_flow<br>",
    ),
    (
        "  |- \u68692\u65e0+\u68691\u6709 -&gt; (1.8,2.6) \u53d6\u68691 -&gt; main_flow<br>",
        "  |- \u68692\u65e0+\u68691\u6709 -&gt; (1.8,2.6) \u53d6\u68691 (p4) -&gt; main_flow<br>",
    ),
    (
        "  |- \u4ec5\u68693\u6709 -&gt; (4.2,2.6) \u53d6\u68693 -&gt; main_flow<br>",
        "  |- \u4ec5\u68693\u6709 -&gt; (4.2,2.6) \u53d6\u68693 (p4) -&gt; main_flow<br>",
    ),
    (
        "  |- \u68692+\u68693\u6709 -&gt; (3.0,2.6) \u53d6\u68692 -&gt; main_flow \u4e0a\u6869\u540e\u518d\u53d63<br>",
        "  |- \u68692+\u68693\u6709 -&gt; (3.0,2.6) \u53d6\u68692 (p4) -&gt; main_flow \u4e0a\u6869\u540e\u518d\u53d63<br>",
    ),
    (
        '<p class="comic-note">\u9884\u5907\u9636\u6bb5<b>\u53ea\u53d6\u4ef6</b>\uff1b\u4e0a\u68692 \u7edf\u4e00\u5728 <span class="code">begin_main_flow()</span> \u91cc\u5b8c\u6210\uff08\u5bfc\u822a <span class="code">(3.0,2.6)</span> \u9884\u5907\u4f4d Process_UpStairs\uff0c\u683c\u5fc3 <span class="code">(3.0,3.8)</span>\uff09\u3002</p>',
        '<p class="comic-note">\u9884\u5907\u9636\u6bb5<b>\u53ea\u53d6\u4ef6</b>\uff08<span class="code">APP_ZONE2_GET_KFS_GROUND_HIGHEST</span> \u2192 \u4e3b\u8f74 p4\uff09\uff1b\u4e0a\u68692 \u7edf\u4e00\u5728 <span class="code">begin_main_flow()</span> \u91cc\u5b8c\u6210\uff08\u5bfc\u822a <span class="code">(3.0,2.6)</span> \u9884\u5907\u4f4d Process_UpStairs\uff0c\u683c\u5fc3 <span class="code">(3.0,3.8)</span>\uff09\u3002</p>',
    ),
]

INSERT_AFTER = '<div class="stage s-main">\n  <div class="sh">\u7b2c2\u5e55'
INSERT_BLOCK = (
    '<div class="stage" style="border-color:var(--orange)">\n'
    '  <div class="sh" style="background:#332200;color:var(--orange);">GetKFS rel \u5206\u6863\uff08\u4e0e app_zone2.h \u4e00\u81f4\uff09</div>\n'
    '  <div class="sb">\n'
    '    <table class="tb">\n'
    '      <tr><th>rel</th><th>\u4f7f\u7528\u9636\u6bb5</th><th>\u4e3b\u8f74\u521d\u4f4d</th></tr>\n'
    '      <tr><td class="code">GROUND_HIGHEST</td><td>Z2_GROUND_PREP \u68691/2/3 \u5730\u9762\u9884\u5907</td><td>p4\uff08\u6700\u9ad8\u6863\uff09</td></tr>\n'
    '      <tr><td class="code">HIGH_TO_LOW</td><td>\u6885\u6797\u4e3b\u5faa\u73af\uff08\u7ad9\u7acb\u6869 &gt; \u90bb\u683c\u6869\uff09</td><td>p0</td></tr>\n'
    '      <tr><td class="code">LOW_TO_HIGH</td><td>\u6885\u6797\u4e3b\u5faa\u73af\uff08\u7ad9\u7acb\u6869 &lt; \u90bb\u683c\u6869\uff09</td><td>p3</td></tr>\n'
    '    </table>\n'
    '  </div>\n'
    '</div>\n\n'
)


def main():
    text = HTML.read_text(encoding="gbk")
    for old, new in REPLACEMENTS:
        if old not in text:
            raise SystemExit(f"missing patch anchor:\n{old[:80]!r}...")
        text = text.replace(old, new, 1)
    if INSERT_BLOCK.strip() not in text:
        if INSERT_AFTER not in text:
            raise SystemExit("insert anchor not found")
        text = text.replace(INSERT_AFTER, INSERT_BLOCK + INSERT_AFTER, 1)
    HTML.write_text(text, encoding="gbk")
    raw = HTML.read_bytes()
    if raw.startswith(b"\xef\xbb\xbf"):
        raise SystemExit("BOM detected after write")
    print("patched", HTML.name)


if __name__ == "__main__":
    main()
