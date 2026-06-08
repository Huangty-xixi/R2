# -*- coding: utf-8 -*-
"""Generate laser_rangefinder_flow.html (GBK). ASCII source + \\u escapes only."""
from pathlib import Path

OUT = Path(__file__).resolve().parent / "laser_rangefinder_flow.html"


def build_html():
  p = []

  def a(*chunks):
    p.append("".join(chunks))

  a(
    '<!DOCTYPE html>\n<html lang="zh-CN">\n<head>\n',
    '<meta http-equiv="Content-Type" content="text/html; charset=GBK">\n',
    '<meta name="viewport" content="width=device-width, initial-scale=1">\n',
    '<title>激光测距链路 - RC26_H7_R2</title>\n',
    '<style>\n',
    ':root{--bg:#0a0e18;--panel:#141e32;--border:#2a4a6e;--text:#eef4ff;--dim:#7a9ab8;',
    '--cyan:#00e5ff;--green:#69f0ae;--orange:#ffab40;--red:#ff5252;--purple:#b388ff;--yellow:#ffee58;}\n',
    '*{box-sizing:border-box;}\n',
    'body{margin:0;font-family:"Microsoft YaHei","SimHei",sans-serif;color:var(--text);',
    'background:radial-gradient(ellipse at 30% 0%,#1a3050 0%,var(--bg) 60%);',
    'padding:18px 22px 48px;line-height:1.55;}\n',
    'h1{margin:0 0 6px;font-size:1.7em;color:var(--cyan);}\n',
    'h2{color:var(--orange);margin:26px 0 12px;font-size:1.12em;border-left:4px solid var(--orange);padding-left:10px;}\n',
    'h3{color:var(--green);margin:16px 0 8px;font-size:1em;}\n',
    '.sub{color:var(--dim);margin:0 0 18px;font-size:0.92em;}\n',
    '.code{font-family:Consolas,monospace;color:var(--cyan);font-size:0.88em;}\n',
    '.banner{background:linear-gradient(135deg,#1a3048,#2a1838);border:1px solid var(--border);',
    'border-radius:10px;padding:14px 18px;margin-bottom:20px;}\n',
    '.stage{background:var(--panel);border:1px solid var(--border);border-radius:12px;margin:16px 0;overflow:hidden;}\n',
    '.sh{background:#0f3460;color:var(--cyan);padding:8px 14px;font-weight:bold;font-size:0.9em;}\n',
    '.sb{padding:14px 18px;}\n',
    'table.tb{width:100%;border-collapse:collapse;font-size:0.88em;}\n',
    'table.tb th,table.tb td{border:1px solid var(--border);padding:8px 10px;text-align:left;}\n',
    'table.tb th{background:#0d1b2a;color:var(--cyan);}\n',
    'table.tb tr:nth-child(even) td{background:rgba(0,0,0,0.12);}\n',
    '.tag{display:inline-block;padding:2px 8px;border-radius:4px;font-size:0.78em;margin-right:6px;}\n',
    '.tag-g{background:#1b3d2a;color:var(--green);}\n',
    '.tag-y{background:#3d3018;color:var(--orange);}\n',
    '.tag-r{background:#3d1b1b;color:var(--red);}\n',
    '.link-chain{display:flex;flex-wrap:wrap;align-items:center;justify-content:center;gap:8px;margin:16px 0;}\n',
    '.lbox{min-width:118px;padding:12px 10px;border-radius:10px;text-align:center;border:2px solid var(--border);',
    'background:#0d1b2a;transition:transform .35s,box-shadow .35s;}\n',
    '.lbox.on{transform:scale(1.06);box-shadow:0 0 22px rgba(0,229,255,.35);border-color:var(--cyan);}\n',
    '.lbox.hw{border-color:var(--purple);}.lbox.isr{border-color:var(--orange);}.lbox.parse{border-color:var(--cyan);}',
    '.lbox.data{border-color:var(--green);}.lbox.app{border-color:var(--yellow);}\n',
    '.lbox .t{font-weight:bold;font-size:0.85em;margin-bottom:4px;}\n',
    '.lbox .d{font-size:0.75em;color:var(--dim);}\n',
    '.larr{color:var(--orange);font-size:1.3em;animation:pulseArr 1s ease-in-out infinite;}\n',
    '@keyframes pulseArr{0%,100%{opacity:.4;}50%{opacity:1;}}\n',
    '.frame-row{display:flex;flex-wrap:wrap;gap:6px;justify-content:center;margin:14px 0;}\n',
    '.byte{min-width:52px;padding:10px 6px;border-radius:8px;background:#0a1220;border:1px solid var(--border);',
    'text-align:center;font-family:Consolas,monospace;font-size:0.82em;opacity:.35;transition:all .25s;}\n',
    '.byte .ix{font-size:0.65em;color:var(--dim);display:block;}\n',
    '.byte .hx{color:var(--yellow);font-weight:bold;}\n',
    '.byte .nm{font-size:0.68em;color:var(--dim);margin-top:4px;}\n',
    '.byte.lit{opacity:1;border-color:var(--cyan);box-shadow:0 0 14px rgba(0,229,255,.4);transform:translateY(-4px);}\n',
    '.byte.lit.chk{border-color:var(--green);box-shadow:0 0 14px rgba(105,240,174,.4);}\n',
    'pre.frame{background:#0a1220;border:1px solid var(--border);border-radius:8px;padding:12px;',
    'overflow-x:auto;font-size:0.8em;color:#b8d4f0;margin:10px 0;}\n',
    '.flow-lane{background:#0d1b2a;border:1px dashed var(--border);border-radius:8px;padding:12px 14px;',
    'font-family:Consolas,monospace;font-size:0.82em;line-height:1.75;}\n',
    '.hl-c{color:var(--cyan);}.hl-g{color:var(--green);}.hl-y{color:var(--orange);}.hl-r{color:var(--red);}\n',
    '.warn{border-left:4px solid var(--orange);background:#2a2010;padding:10px 14px;margin:12px 0;border-radius:0 8px 8px 0;}\n',
    '.ctrl{margin:12px 0;display:flex;flex-wrap:wrap;gap:10px;align-items:center;}\n',
    'button{cursor:pointer;padding:8px 16px;border-radius:8px;border:1px solid var(--cyan);',
    'background:#0f3460;color:var(--cyan);font-weight:bold;}\n',
    'button:hover{background:#1a5080;}\n',
    '#pktStatus{color:var(--green);font-size:0.9em;min-height:1.4em;}\n',
    'ul{margin:8px 0;padding-left:22px;}li{margin:4px 0;}\n',
    '</style></head><body>\n',
  )

  a(
    '<h1>激光测距链路（TinyF \xb7 UART7）</h1>\n',
    '<p class="sub">RC26_H7_R2 &nbsp;|&nbsp; <span class="code">sensor.c</span> / <span class="code">sensor.h</span> '
    '/ <span class="code">stm32h7xx_it.c</span> &nbsp;|&nbsp; <span class="code">laser1</span> '
    '/ <span class="code">laser1_diag</span></p>\n',
    '<div class="banner"><strong>一句话：</strong> '
    'TinyF 上电连续发 <b>ASCII 帧</b>：'
    '<span class="code">0x20</span> + 距离(1..5) + <span class="code">,&nbsp;</span> + 置信度(1..2) + <span class="code">0x0A</span>；'
    'MCU 状态机逐字节解析写入 <span class="code">laser1</span>（四状态）。</div>\n',
  )

  # animated link chain
  a(
    '<div class="stage"><div class="sh">动画：数据链路（点击播放一帧）</div><div class="sb">\n',
    '<div class="ctrl"><button type="button" id="btnPlay">&gt; 播放一帧</button>',
    '<button type="button" id="btnAuto">自动播放</button>',
    '<button type="button" id="btnStop">停止</button></div>\n',
    '<div id="pktStatus"></div>\n',
    '<div class="link-chain" id="linkChain">\n',
    '<div class="lbox hw" data-step="0"><div class="t">TinyF</div><div class="d">UART TX</div></div>',
    '<span class="larr">&rarr;</span>',
    '<div class="lbox hw" data-step="1"><div class="t">PE7 RX</div><div class="d">115200 8N1</div></div>',
    '<span class="larr">&rarr;</span>',
    '<div class="lbox isr" data-step="2"><div class="t">UART7_IRQHandler</div><div class="d">RDR&amp;rarr;OnRxByte</div></div>',
    '<span class="larr">&rarr;</span>',
    '<div class="lbox parse" data-step="3"><div class="t">sensor.c</div><div class="d">State machine</div></div>',
    '<span class="larr">&rarr;</span>',
    '<div class="lbox data" data-step="4"><div class="t">laser1</div><div class="d">mm / ready</div></div>',
    '<span class="larr">&rarr;</span>',
    '<div class="lbox app" data-step="5"><div class="t">Plan B</div><div class="d">sudden_increase</div></div>',
    '</div>\n',
    '<h3>实例帧（距离 2882mm / 置信度 58）</h3>\n',
    '<div class="frame-row" id="frameBytes"></div>\n',
    '<p style="color:var(--dim);font-size:0.85em">ASCII: <span class="code"> 2882, 58\\n</span> &nbsp;|&nbsp; 长度随数字位变化，以 <span class="code">0x0A</span> 为帧尾</p>\n',
    '</div></div>\n',
  )

  # hardware table
  a(
    '<div class="stage"><div class="sh">硬件与初始化</div><div class="sb">',
    '<table class="tb"><tr><th>项目</th><th>值</th><th>源文件</th></tr>',
    '<tr><td>串口</td><td>UART7, PE7=RX（仅收）</td><td><span class="code">usart.c</span></td></tr>',
    '<tr><td>波特率</td><td>115200, 8N1</td><td><span class="code">MX_UART7_Init</span></td></tr>',
    '<tr><td>上电序</td><td>延时后再 Init + <span class="code">Laser_Init</span></td><td><span class="code">main.c</span></td></tr>',
    '<tr><td>RX 中断</td><td><span class="code">RXNEIE</span>, 自定义 ISR，诊断计数器内联</td><td><span class="code">stm32h7xx_it.c</span></td></tr>',
    '<tr><td>保活</td><td><span class="code">Laser_UART7_RxIrqSanityCheck</span></td><td><span class="code">Sensor_Task.c</span></td></tr>',
    '</table></div></div>\n',
  )

  # frame format
  a(
    '<div class="stage"><div class="sh">TinyF 数据格式（手册）</div><div class="sb">',
    '<pre class="frame">',
    '[0x20] + distance(1..5 ASCII) + [0x2C 0x20] + confidence(1..2 ASCII) + [0x0A]\n',
    '例: 20 33 32 37 2C 20 36 31 0A  -&gt;  " 327, 61\\n"  distance=327  conf=61\n',
    '例: 20 32 38 38 32 2C 20 35 38 0A  -&gt;  " 2882, 58\\n" distance=2882 conf=58',
    '</pre>\n',
    '<table class="tb"><tr><th>字段</th><th>值</th><th>说明</th></tr>',
    '<tr><td>帧头</td><td><span class="code">0x20</span></td><td>空格</td></tr>',
    '<tr><td>距离</td><td>1~5 字节</td><td>ASCII 数字，单位 mm</td></tr>',
    '<tr><td>分隔</td><td><span class="code">0x2C 0x20</span></td><td>", "</td></tr>',
    '<tr><td>置信度</td><td>1~2 字节</td><td>ASCII 0..62</td></tr>',
    '<tr><td>帧尾</td><td><span class="code">0x0A</span></td><td>LF（无 CR 亦可）</td></tr>',
    '</table>\n',
    '<div class="flow-lane">',
    '<span class="hl-c">sensor_uart7_on_rx_byte</span> (状态机)<br>',
    'ST_WAIT_HEAD(0x20) → ST_DIST(数字→0x2C) → ST_SEP(0x20) → ST_CONF(数字→0x0A)<br>',
    '逐位累加距离/置信度 → 校验 → <span class="hl-g">laser1</span>',
    '</div>\n',
    '<div class="warn"><span class="tag tag-y">注</span> ',
    '非 0x59 二进制；距离 20..4000mm、置信度 1..62 才置 ready',
    '</div></div></div>\n',
  )

  # laser1 + diag
  a(
    '<div class="stage"><div class="sh">laser1 / laser1_diag（Keil Watch）</div><div class="sb">',
    '<table class="tb"><tr><th colspan="2">laser1</th></tr>',
    '<tr><td><span class="code">distance</span></td><td>距离 mm（20..4000 有效）</td></tr>',
    '<tr><td><span class="code">confidence</span></td><td>帧内 ASCII，1..62</td></tr>',
    '<tr><td><span class="code">ready</span></td><td>至少一帧有效解析</td></tr>',
    '<tr><td><span class="code">sudden_increase</span></td><td>相邻有效帧 增&gt;=100mm</td></tr>',
    '</table><br>',
    '<table class="tb"><tr><th>计数器</th><th>含义</th></tr>',
    '<tr><td><span class="code">rx_byte_cnt</span></td><td>收到字节数（链路通）</td></tr>',
    '<tr><td><span class="code">frame_ok_cnt</span></td><td>解析成功且量程合格</td></tr>',
    '<tr><td><span class="code">parse_fail_cnt</span></td><td>帧格错/超长/解析失败</td></tr>',
    '<tr><td><span class="code">irq_cnt</span></td><td>UART7 中断次数</td></tr>',
    '<tr><td><span class="code">ore_cnt / fe_cnt</span></td><td>溢出 / 帧错误</td></tr>',
    '</table></div></div>\n',
  )

  # software flow
  a(
    '<div class="stage"><div class="sh">软件调用链</div><div class="sb">',
    '<div class="flow-lane">',
    '<span class="hl-y">main</span> MX_UART7_Init &amp;rarr; delay &amp;rarr; MX_UART7_Init &amp;rarr; <span class="hl-c">Laser_Init</span><br>',
    '<span class="hl-y">ISR</span> UART7_IRQHandler &amp;rarr; 循环读ISR标志 → Laser_UART7_OnRxByte → sensor_uart7_on_rx_byte<br>',
    '<span class="hl-y">Task</span> Sensor_Task &amp;rarr; Laser_UART7_RxIrqSanityCheck (~2ms)<br>',
    '<span class="hl-g">Plan B</span> Process_DownStairs &amp;rarr; Laser_GetSuddenIncrease / ClearSuddenIncrease',
    '</div>',
    '<div class="warn"><span class="tag tag-r">PLAN=0</span> 默认下台阶不用激光；解析仍运行。',
    ' Plan B: <span class="code">-DPROCESS_FLOW_DOWNSTAIRS_PLAN=1</span></div>',
    '</div></div>\n',
  )

  # API
  a(
    '<div class="stage"><div class="sh">API 速查</div><div class="sb"><table class="tb">',
    '<tr><th>函数</th><th>说明</th></tr>',
    '<tr><td><span class="code">Laser_Init</span></td><td>flush + 开 RXNEIE（不调 HAL_Abort）</td></tr>',
    '<tr><td><span class="code">Laser_UART7_OnRxByte</span></td><td>ISR 每字节 → 状态机</td></tr>',
    '<tr><td><span class="code">Laser_GetSuddenIncrease</span></td><td>读突增标志</td></tr>',
    '<tr><td><span class="code">Laser_ClearSuddenIncrease</span></td><td>清标志</td></tr>',
  )

  a(
    '</table><p style="text-align:center;color:var(--dim);font-size:0.8em;margin-top:16px">',
    '<span class="code">_gen_laser_rangefinder_gbk.py</span> &rarr; laser_rangefinder_flow.html | GBK</p>\n',
  )

  # JS (ASCII only)
  a(
    '<script>\n',
    '(function(){\n',
    'var FRAME=[0x20,0x32,0x38,0x38,0x32,0x2C,0x20,0x35,0x38,0x0A];\n',
    'var NAMES=["SPC","d0","d1","d2","d3",",","SPC","c0","c1","LF"];\n',
    'var i,b,el=document.getElementById("frameBytes"),n=FRAME.length;\n',
    'for(i=0;i<n;i++){\n',
    ' b=document.createElement("div");b.className="byte";b.id="b"+i;\n',
    ' b.innerHTML=\'<span class="ix">B\'+i+\'</span><span class="hx">\'+FRAME[i].toString(16).toUpperCase().padStart(2,"0")+\'</span><span class="nm">\'+NAMES[i]+\'</span>\';\n',
    ' el.appendChild(b);\n',
    '}\n',
    'var chain=document.querySelectorAll("#linkChain .lbox");\n',
    'var status=document.getElementById("pktStatus");\n',
    'var timer=null,auto=false;\n',
    'function clearLit(){for(i=0;i<n;i++){var e=document.getElementById("b"+i);if(e)e.className="byte";}\n',
    ' for(i=0;i<chain.length;i++) chain[i].classList.remove("on");}\n',
    'function playFrame(){\n',
    ' clearLit();var step=0;\n',
    ' function tick(){\n',
    '  if(step<chain.length){chain[step].classList.add("on");}\n',
    '  if(step>=2 && step-2<n){\n',
    '   var bi=step-2;document.getElementById("b"+bi).className="byte lit"+(bi===n-1?" chk":"");\n',
    '  }\n',
    '  if(step===2) status.textContent="ISR: byte loop, diag counters inline";\n',
    '  if(step===3) status.textContent="State machine: ST_WAIT_HEAD→ST_DIST→ST_SEP→ST_CONF → laser1";\n',
    '  if(step===4) status.textContent="laser1 ready=1 frame_ok_cnt++";\n',
    '  if(step===5) status.textContent="Plan B: sudden_increase if delta>=100mm";\n',
    '  step++;\n',
    '  if(step<=6) setTimeout(tick,380);\n',
    '  else if(auto) setTimeout(function(){playFrame();},1200);\n',
    ' }\n',
    ' tick();\n',
    '}\n',
    'document.getElementById("btnPlay").onclick=function(){auto=false;playFrame();};\n',
    'document.getElementById("btnAuto").onclick=function(){auto=true;playFrame();};\n',
    'document.getElementById("btnStop").onclick=function(){auto=false;clearInterval(timer);clearLit();status.textContent="stopped";};\n',
    '})();\n',
    '</script></body></html>\n',
  )

  return "".join(p)


def main():
    content = build_html()
    OUT.write_text(content, encoding="gbk")
    raw = OUT.read_bytes()
    if raw.startswith(b"\xef\xbb\xbf"):
        raise SystemExit("BOM detected")
    if b"charset=GBK" not in raw[:800]:
        raise SystemExit("meta GBK missing")
    print("wrote", OUT, len(raw), "bytes")


if __name__ == "__main__":
    main()