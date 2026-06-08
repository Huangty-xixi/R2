# -*- coding: utf-8 -*-
"""Generate animated downstairs_flow.html (GBK). ASCII + \\u escapes only."""
from pathlib import Path

OUT = Path(__file__).resolve().parent / "downstairs_flow.html"


def build():
    parts = []

    def u(*chunks):
        parts.append("".join(chunks))

    # CSS + structure (JS is ASCII-only)
    u(
        '<!DOCTYPE html>\n<html lang="zh-CN">\n<head>\n',
        '<meta http-equiv="Content-Type" content="text/html; charset=GBK">\n',
        '<meta name="viewport" content="width=device-width, initial-scale=1">\n',
        '<title>\u4e0b\u53f0\u9636\u52a8\u753b\u6d41\u7a0b</title>\n',
        '<style>\n',
        ':root{--bg:#0a0e1a;--panel:#141c2e;--border:#2d4a6e;--text:#eef4ff;--dim:#7a9ab8;',
        '--cyan:#00e5ff;--green:#69f0ae;--orange:#ffab40;--red:#ff5252;--purple:#b388ff;--yellow:#ffee58;}\n',
        '*{box-sizing:border-box;}\n',
        'body{margin:0;font-family:"Microsoft YaHei","SimHei",sans-serif;color:var(--text);',
        'background:radial-gradient(ellipse at 50% 0%,#1a2840 0%,var(--bg) 55%);min-height:100vh;padding:16px 20px 40px;}\n',
        'h1{margin:0;font-size:1.75em;background:linear-gradient(90deg,var(--cyan),var(--purple));',
        '-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text;',
        'animation:titleGlow 3s ease-in-out infinite;}\n',
        '@keyframes titleGlow{0%,100%{filter:drop-shadow(0 0 8px rgba(0,229,255,.4));}',
        '50%{filter:drop-shadow(0 0 18px rgba(179,136,255,.6));}}\n',
        '.sub{color:var(--dim);margin:8px 0 20px;font-size:0.9em;}\n',
        '.code{font-family:Consolas,monospace;color:var(--cyan);font-size:0.88em;}\n',
        '.plan-tabs{display:flex;flex-wrap:wrap;gap:10px;margin:16px 0;}\n',
        '.plan-tab{cursor:pointer;padding:10px 18px;border-radius:999px;border:2px solid var(--border);',
        'background:rgba(0,0,0,.25);font-weight:bold;transition:all .35s;user-select:none;}\n',
        '.plan-tab:hover{transform:translateY(-2px);box-shadow:0 6px 20px rgba(0,0,0,.4);}\n',
        '.plan-tab.on-a{border-color:var(--green);color:var(--green);box-shadow:0 0 20px rgba(105,240,174,.35);}\n',
        '.plan-tab.on-b{border-color:var(--orange);color:var(--orange);box-shadow:0 0 20px rgba(255,171,64,.35);}\n',
        '.plan-tab.on-c{border-color:var(--purple);color:var(--purple);box-shadow:0 0 20px rgba(179,136,255,.35);}\n',
        '.plan-panel{display:none;animation:fadeIn .5s ease;}\n',
        '.plan-panel.active{display:block;}\n',
        '@keyframes fadeIn{from{opacity:0;transform:translateY(12px);}to{opacity:1;transform:none;}}\n',
        '.scene-wrap{background:var(--panel);border:1px solid var(--border);border-radius:16px;',
        'padding:20px;margin:12px 0;overflow:hidden;}\n',
        '.scene-title{font-size:1.05em;font-weight:bold;margin-bottom:14px;}\n',
        '.theater{position:relative;height:200px;background:linear-gradient(180deg,#1a3050 0%,#0d1828 70%,#1a2030 100%);',
        'border-radius:12px;border:1px solid var(--border);overflow:hidden;margin-bottom:20px;}\n',
        '.ground{position:absolute;bottom:0;left:0;right:0;height:42px;background:linear-gradient(180deg,#2a3548,#1a2230);}\n',
        '.step-edge{position:absolute;bottom:42px;right:18%;width:120px;height:28px;',
        'background:linear-gradient(135deg,#4a5568,#2d3748);transform:skewX(-12deg);',
        'border-radius:4px 4px 0 0;box-shadow:0 -4px 12px rgba(0,0,0,.5);}\n',
        '.step-edge::after{content:"";position:absolute;top:-18px;right:0;width:100%;height:18px;',
        'background:inherit;transform:skewX(12deg);}\n',
        '.robot{position:absolute;bottom:48px;left:12%;width:56px;height:48px;transition:left 1.2s ease;}\n',
        '.robot-body{width:52px;height:32px;background:linear-gradient(145deg,#4fc3f7,#1976d2);',
        'border-radius:8px;position:relative;box-shadow:0 4px 12px rgba(0,0,0,.5);}\n',
        '.robot-wheel{position:absolute;bottom:-8px;width:14px;height:14px;background:#333;',
        'border-radius:50%;border:2px solid #666;animation:wheelSpin .6s linear infinite;}\n',
        '.robot-wheel.w1{left:6px;}.robot-wheel.w2{right:6px;}\n',
        '@keyframes wheelSpin{to{transform:rotate(360deg);}}\n',
        '.lift-mast{position:absolute;left:50%;bottom:28px;width:6px;height:var(--lift-h,24px);',
        'background:#90a4ae;margin-left:-3px;transition:height .8s ease;border-radius:3px;}\n',
        '.lift-head{position:absolute;left:50%;bottom:calc(var(--lift-h,24px) + 22px);width:20px;height:12px;',
        'background:var(--orange);margin-left:-10px;border-radius:3px;transition:bottom .8s ease;}\n',
        '.laser-beam{position:absolute;bottom:55px;left:calc(12% + 60px);width:0;height:3px;',
        'background:linear-gradient(90deg,var(--red),transparent);opacity:0;transform-origin:left;}\n',
        '.pitch-gauge{position:absolute;top:12px;left:12px;width:80px;height:80px;border:3px solid var(--border);',
        'border-radius:50%;background:rgba(0,0,0,.3);}\n',
        '.pitch-needle{position:absolute;bottom:50%;left:50%;width:4px;height:36px;background:var(--yellow);',
        'transform-origin:bottom center;margin-left:-2px;border-radius:2px;}\n',
        '.demo-label{position:absolute;top:10px;right:12px;font-size:0.75em;color:var(--cyan);',
        'animation:blink 1.2s step-end infinite;}\n',
        '@keyframes blink{50%{opacity:.3;}}\n',
        '.comic-row{display:flex;flex-wrap:wrap;align-items:stretch;gap:0;justify-content:center;}\n',
        '.comic-step{flex:1;min-width:100px;max-width:160px;padding:8px 4px;text-align:center;position:relative;}\n',
        '.step-card{background:#0d1b2a;border:2px solid var(--border);border-radius:12px;padding:12px 8px;',
        'min-height:100px;transition:all .4s;position:relative;overflow:hidden;}\n',
        '.step-card::before{content:"";position:absolute;inset:0;background:linear-gradient(135deg,transparent 60%,rgba(255,255,255,.03));}\n',
        '.step-card.active{border-color:var(--cyan);box-shadow:0 0 24px rgba(0,229,255,.45);transform:scale(1.05);z-index:2;}\n',
        '.step-icon{font-size:1.8em;line-height:1;margin-bottom:6px;display:block;}\n',
        '.step-name{font-size:0.78em;font-weight:bold;color:var(--text);}\n',
        '.step-detail{font-size:0.68em;color:var(--dim);margin-top:4px;line-height:1.35;}\n',
        '.arrow-between{align-self:center;color:var(--orange);font-size:1.4em;animation:arrowPulse 1s ease-in-out infinite;}\n',
        '@keyframes arrowPulse{0%,100%{opacity:.4;transform:translateX(0);}50%{opacity:1;transform:translateX(4px);}}\n',
        '.ctrl{margin-top:14px;display:flex;flex-wrap:wrap;gap:10px;align-items:center;}\n',
        '.btn{cursor:pointer;padding:8px 16px;border-radius:8px;border:none;font-weight:bold;',
        'background:linear-gradient(145deg,#1976d2,#0d47a1);color:#fff;transition:transform .2s;}\n',
        '.btn:hover{transform:scale(1.05);}\n',
        '.btn.sec{background:#2a3548;color:var(--cyan);border:1px solid var(--border);}\n',
        '.param-table{width:100%;border-collapse:collapse;font-size:0.82em;margin-top:12px;}\n',
        '.param-table th,.param-table td{border:1px solid var(--border);padding:6px 8px;}\n',
        '.param-table th{background:#0a1220;color:var(--cyan);}\n',
        '/* theater motion modes */\n',
        '.theater.mode-back .robot{animation:robotBack 2s ease-in-out infinite alternate;}\n',
        '.theater.mode-fwd .robot{animation:robotFwd 2s ease-in-out infinite alternate;}\n',
        '.theater.mode-lift .lift-mast{--lift-h:70px;}.theater.mode-lift .lift-head{bottom:94px;}\n',
        '.theater.mode-fall .lift-mast{--lift-h:12px;animation:liftShake .4s ease;}\n',
        '.theater.mode-laser .laser-beam{width:140px;opacity:1;animation:laserPing .8s ease-in-out infinite;}\n',
        '.theater.mode-pitch .pitch-needle{animation:pitchSway 2.5s ease-in-out infinite;}\n',
        '@keyframes robotBack{from{left:12%;}to{left:6%;}}\n',
        '@keyframes robotFwd{from{left:8%;}to{left:14%;}}\n',
        '@keyframes laserPing{0%,100%{opacity:.5;width:100px;}50%{opacity:1;width:160px;}}\n',
        '@keyframes pitchSway{0%,100%{transform:rotate(-25deg);}40%{transform:rotate(15deg);}70%{transform:rotate(-8deg);}}\n',
        '@keyframes liftShake{0%,100%{transform:translateY(0);}50%{transform:translateY(4px);}}\n',
        '.compare-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px;margin-top:16px;}\n',
        '.compare-card{border-radius:12px;padding:14px;border:2px solid var(--border);text-align:center;}\n',
        '.compare-card.a{border-color:var(--green);}.compare-card.b{border-color:var(--orange);}',
        '.compare-card.c{border-color:var(--purple);}\n',
        '</style>\n</head>\n<body>\n',
        '<h1>\u4e0b\u53f0\u9636 \u52a8\u753b\u6f14\u793a</h1>\n',
        '<p class="sub"><span class="code">Process_DownStairs()</span> &nbsp;|&nbsp; ',
        '<span class="code">PROCESS_FLOW_DOWNSTAIRS_PLAN</span> \u5207\u6362\u65b9\u6848 &nbsp;|&nbsp; ',
        '\u70b9\u51fb\u6b65\u9aa4\u5361\u7247\u6216\u300c\u81ea\u52a8\u64ad\u653e\u300d\u770b\u52a8\u753b</p>\n',
        '<div class="plan-tabs">\n',
        '<div class="plan-tab on-a active" data-plan="a">&#128994; Plan A \u4fef\u4ef0\u611f\u77e5</div>\n',
        '<div class="plan-tab on-b" data-plan="b">&#128308; Plan B \u6fc0\u5149\u7a81\u589e</div>\n',
        '<div class="plan-tab on-c" data-plan="c">&#128995; Plan C \u5148\u524d\u540e\u9000</div>\n',
        '</div>\n',
    )

    # Plan A panel
    u(
        '<div id="panel-a" class="plan-panel active">\n',
        '<div class="scene-wrap"><div class="scene-title" style="color:var(--green)">',
        'Plan A = 0 &nbsp; \u5feb\u62ac + \u5012\u8f66 + IMU \u4fef\u4ef0\u8d77\u843d</div>\n',
        '<div id="theater-a" class="theater mode-pitch">\n',
        '<span class="demo-label" id="demo-label-a">\u6f14\u793a\u4e2d...</span>\n',
        '<div class="pitch-gauge"><div class="pitch-needle" id="needle-a"></div></div>\n',
        '<div class="step-edge"></div><div class="ground"></div>\n',
        '<div class="robot" id="robot-a"><div class="lift-mast"></div><div class="lift-head"></div>',
        '<div class="robot-body"><span class="robot-wheel w1"></span><span class="robot-wheel w2"></span></div></div>\n',
        '</div>\n',
        '<div class="comic-row" id="steps-a">\n',
        _step_cards_a(),
        '</div>\n',
        '<div class="ctrl"><button class="btn" onclick="autoPlay(\'a\')">&#9654; \u81ea\u52a8\u64ad\u653e</button>',
        '<button class="btn sec" onclick="stopPlay()">&#9632; \u505c\u6b62</button></div>\n',
        _param_table_a(),
        '</div></div>\n',
    )

    # Plan B panel
    u(
        '<div id="panel-b" class="plan-panel">\n',
        '<div class="scene-wrap"><div class="scene-title" style="color:var(--orange)">',
        'Plan B = 1 &nbsp; Plan A \u4fef\u4ef0 + wait \u540e\u5012\u9000(vy=-20) + \u6fc0\u5149/3s \u2192 fall_fast</div>\n',
        '<div id="theater-b" class="theater">\n',
        '<span class="demo-label" id="demo-label-b">\u6f14\u793a\u4e2d...</span>\n',
        '<div class="laser-beam" id="laser-b"></div>\n',
        '<div class="step-edge"></div><div class="ground"></div>\n',
        '<div class="robot" id="robot-b"><div class="lift-mast"></div><div class="lift-head"></div>',
        '<div class="robot-body"><span class="robot-wheel w1"></span><span class="robot-wheel w2"></span></div></div>\n',
        '</div>\n',
        '<div class="comic-row" id="steps-b">\n',
        _step_cards_b(),
        '</div>\n',
        '<div class="ctrl"><button class="btn" onclick="autoPlay(\'b\')">&#9654; \u81ea\u52a8\u64ad\u653e</button>',
        '<button class="btn sec" onclick="stopPlay()">&#9632; \u505c\u6b62</button></div>\n',
        _param_table_b(),
        '</div></div>\n',
    )

    # Plan C panel
    u(
        '<div id="panel-c" class="plan-panel">\n',
        '<div class="scene-wrap"><div class="scene-title" style="color:var(--purple)">',
        'Plan C = 2 &nbsp; \u5168\u7a0b\u8ba1\u65f6\uff1a\u524d\u8fdb \u2192 \u540e\u9000 \u2192 \u62ac \u2192 \u518d\u9000 \u2192 \u964d</div>\n',
        '<div id="theater-c" class="theater">\n',
        '<span class="demo-label" id="demo-label-c">\u6f14\u793a\u4e2d...</span>\n',
        '<div class="step-edge"></div><div class="ground"></div>\n',
        '<div class="robot" id="robot-c"><div class="lift-mast"></div><div class="lift-head"></div>',
        '<div class="robot-body"><span class="robot-wheel w1"></span><span class="robot-wheel w2"></span></div></div>\n',
        '</div>\n',
        '<div class="comic-row" id="steps-c">\n',
        _step_cards_c(),
        '</div>\n',
        '<div class="ctrl"><button class="btn" onclick="autoPlay(\'c\')">&#9654; \u81ea\u52a8\u64ad\u653e</button>',
        '<button class="btn sec" onclick="stopPlay()">&#9632; \u505c\u6b62</button></div>\n',
        _param_table_c(),
        '</div></div>\n',
    )

    # Compare + JS
    u(
        '<div class="scene-wrap"><div class="scene-title">\u4e09\u65b9\u6848\u4e00\u773c\u5bf9\u6bd4</div>\n',
        '<div class="compare-grid">\n',
        '<div class="compare-card a"><div style="font-size:2em">&#128200;</div><b>Plan A</b><br>',
        '<span style="font-size:0.85em;color:var(--dim)">|pitch| \u8d77\u843d<br>\u4e0d\u8981\u6fc0\u5149</span></div>\n',
        '<div class="compare-card b"><div style="font-size:2em">&#128225;</div><b>Plan B</b><br>',
        '<span style="font-size:0.85em;color:var(--dim)">A\u4fef\u4ef0+wait<br>vy=-20 \u6fc0\u5149/3s</span></div>\n',
        '<div class="compare-card c"><div style="font-size:2em">&#9201;</div><b>Plan C</b><br>',
        '<span style="font-size:0.85em;color:var(--dim)">\u7eaf\u8ba1\u65f6<br>\u5148\u524d\u540e\u9000</span></div>\n',
        '</div></div>\n',
        '<p style="text-align:center;color:var(--dim);font-size:0.8em;margin-top:24px">',
        '<span class="code">_gen_downstairs_flow_gbk.py</span> | GBK | \u5165\u53e3: Can_Task / app_zone2</p>\n',
        _javascript(),
        '</body></html>\n',
    )

    return "".join(parts)


def _step_card(icon, name, detail):
    return (
        '<div class="comic-step"><div class="step-card" data-detail="">'
        f'<span class="step-icon">{icon}</span>'
        f'<div class="step-name">{name}</div>'
        f'<div class="step-detail">{detail}</div></div></div>'
        '<span class="arrow-between">&rsaquo;</span>'
    )


def _step_cards_a():
    s = ""
    steps = [
        ("&#128640;", "idle", "raise_fast + vy=-50"),
        ("&#128200;", "pitch\u2191", "|pitch|-base &ge;10\u00b0"),
        ("&#128201;", "pitch\u2193", "\u8fde\u7eed\u56de\u843d x3"),
        ("&#9208;", "wait", "500ms"),
        ("&#9194;", "\u518d\u9000", "-40 x 1.2s"),
        ("&#128763;", "fall_fast", "100ms done"),
    ]
    for i, (ic, nm, dt) in enumerate(steps):
        s += '<div class="comic-step"><div class="step-card">'
        s += f'<span class="step-icon">{ic}</span><div class="step-name">{nm}</div>'
        s += f'<div class="step-detail">{dt}</div></div></div>'
        if i < len(steps) - 1:
            s += '<span class="arrow-between">&rsaquo;</span>'
    return s


def _step_cards_b():
    s = ""
    steps = [
        ("&#128640;", "\u5feb\u62ac", "+\u4fef\u4ef0\u5012\u9000"),
        ("&#128200;", "pitch\u2191\u2193", "\u540c Plan A"),
        ("&#9208;", "wait", "500ms"),
        ("&#9194;", "\u5012\u9000", "vy=-20"),
        ("&#128225;", "\u7a81\u589e/3s", "\u505c\u8f66"),
        ("&#128763;", "fall_fast", "done"),
    ]
    for i, (ic, nm, dt) in enumerate(steps):
        s += '<div class="comic-step"><div class="step-card">'
        s += f'<span class="step-icon">{ic}</span><div class="step-name">{nm}</div>'
        s += f'<div class="step-detail">{dt}</div></div></div>'
        if i < len(steps) - 1:
            s += '<span class="arrow-between">&rsaquo;</span>'
    return s


def _step_cards_c():
    s = ""
    steps = [
        ("&#9193;", "\u524d\u8fdb", "+10 x 3s"),
        ("&#9194;", "\u540e\u9000", "-10 x 3.7s"),
        ("&#11014;", "\u62ac\u5347", "hold 1.5s"),
        ("&#9194;", "\u518d\u9000", "-20 x 3.5s"),
        ("&#9208;", "\u7b49\u5f85", "1s"),
        ("&#11015;", "\u964d\u5e95", "fall"),
    ]
    for i, (ic, nm, dt) in enumerate(steps):
        s += '<div class="comic-step"><div class="step-card">'
        s += f'<span class="step-icon">{ic}</span><div class="step-name">{nm}</div>'
        s += f'<div class="step-detail">{dt}</div></div></div>'
        if i < len(steps) - 1:
            s += '<span class="arrow-between">&rsaquo;</span>'
    return s


def _param_table_a():
    return (
        '<table class="param-table"><tr><th colspan="2">g_process_downstairs_tune</th></tr>'
        '<tr><td>vy_backward</td><td>-50</td></tr>'
        '<tr><td>pitch rise/fall th</td><td>10\u00b0 x 3\u6b21</td></tr>'
        '<tr><td>fast_raise_back_ms</td><td>1200</td></tr></table>'
    )


def _param_table_b():
    return (
        '<table class="param-table"><tr><th colspan="2">Plan B \u53c2\u6570</th></tr>'
        '<tr><td>g_process_downstairs_tune</td><td>\u4fef\u4ef0\u6bb5\uff08\u540c A\uff09</td></tr>'
        '<tr><td>vy_rev</td><td>-20</td></tr>'
        '<tr><td>laser_rev_timeout_ms</td><td>3000</td></tr>'
        '<tr><td>laser</td><td>sudden_increase &ge;100mm</td></tr></table>'
    )


def _param_table_c():
    return (
        '<table class="param-table"><tr><th colspan="2">g_process_downstairs_plan_c_tune</th></tr>'
        '<tr><td>vy_fwd_ms</td><td>3000</td></tr>'
        '<tr><td>vy_rev_first_ms</td><td>3700</td></tr></table>'
    )


def _javascript():
    return r"""
<script>
(function(){
  var playTimer=null;
  var configs={
    a:{
      theater:'theater-a',label:'demo-label-a',steps:'steps-a',
      modes:['mode-lift','mode-back','mode-pitch','mode-pitch','mode-back','mode-fall'],
      labels:['\u5feb\u62ac\u5347+\u540e\u9000','\u4fef\u4ef0\u89d2\u5ea6\u4e0a\u5347','\u4fef\u4ef0\u56de\u843d\u786e\u8ba4','\u505c\u7a33 500ms','\u518d\u540e\u9000 1.2s','\u5feb\u964d\u5b8c\u6210']
    },
    b:{
      theater:'theater-b',label:'demo-label-b',steps:'steps-b',
      modes:['mode-lift','mode-pitch','','mode-back','mode-laser','mode-fall'],
      labels:['\u5feb\u62ac+\u4fef\u4ef0\u5012\u9000','\u4fef\u4ef0\u8d77\u843d\u786e\u8ba4','wait 500ms','\u5012\u9000 vy=-20','\u7a81\u589e\u6216 3s\u8d85\u65f6','\u505c\u8f66 fall_fast']
    },
    c:{
      theater:'theater-c',label:'demo-label-c',steps:'steps-c',
      modes:['mode-fwd','mode-back','mode-lift','mode-back','','mode-fall'],
      labels:['\u524d\u8fdb 3s','\u540e\u9000 3.7s','\u62ac\u5347\u4fdd\u6301','\u518d\u9000 3.5s','\u7b49\u5f85 1s','\u964d\u5e95']
    }
  };

  document.querySelectorAll('.plan-tab').forEach(function(tab){
    tab.addEventListener('click',function(){
      stopPlay();
      document.querySelectorAll('.plan-tab').forEach(function(t){t.classList.remove('active');});
      document.querySelectorAll('.plan-panel').forEach(function(p){p.classList.remove('active');});
      tab.classList.add('active');
      var p=tab.getAttribute('data-plan');
      document.getElementById('panel-'+p).classList.add('active');
    });
  });

  document.querySelectorAll('.comic-row').forEach(function(row){
    row.querySelectorAll('.step-card').forEach(function(card,idx){
      card.style.cursor='pointer';
      card.addEventListener('click',function(){
        var plan=row.id.replace('steps-','');
        highlightStep(plan,idx);
      });
    });
  });

  function setTheaterModes(theater,modeStr){
    theater.className='theater'+(modeStr?(' '+modeStr):'');
  }

  window.highlightStep=function(plan,idx){
    var c=configs[plan];
    if(!c)return;
    var theater=document.getElementById(c.theater);
    var label=document.getElementById(c.label);
    var row=document.getElementById(c.steps);
    var cards=row.querySelectorAll('.step-card');
    cards.forEach(function(card,i){card.classList.toggle('active',i===idx);});
    setTheaterModes(theater,c.modes[idx]||'');
    if(label&&c.labels[idx])label.textContent=c.labels[idx];
  };

  window.autoPlay=function(plan){
    stopPlay();
    var c=configs[plan];
  var idx=0;
  var n=document.getElementById(c.steps).querySelectorAll('.step-card').length;
  highlightStep(plan,0);
  playTimer=setInterval(function(){
    idx=(idx+1)%n;
    highlightStep(plan,idx);
  },2200);
  };

  window.stopPlay=function(){
    if(playTimer){clearInterval(playTimer);playTimer=null;}
  };
})();
</script>
"""


def main():
    content = build()
    OUT.write_text(content, encoding="gbk")
    if OUT.read_bytes().startswith(b"\xef\xbb\xbf"):
        raise SystemExit("BOM")
    print("wrote", OUT.name, len(OUT.read_bytes()), "bytes")


if __name__ == "__main__":
    main()
