"""Self-contained local replay viewer; no network or hardware calls."""
import base64
import json
from pathlib import Path
from .session import replay_session


def export_replay_html(directory,output):
    directory=Path(directory);session=replay_session(directory)
    images={}
    for record in session['records']:
        if record['kind']=='image':
            relative=record['asset']['path'];path=directory/relative
            if path.is_file():images[relative]='data:image/png;base64,'+base64.b64encode(path.read_bytes()).decode('ascii')
    payload=json.dumps({'session':session,'images':images},ensure_ascii=False,allow_nan=False).replace('<',chr(92)+'u003c')
    template='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>五子棋会话回放</title>
<style>body{font:16px system-ui;margin:24px;background:#f4f3ef;color:#182520}header{max-width:1100px;margin:auto}
main{display:flex;gap:24px;flex-wrap:wrap;margin:20px auto;max-width:1100px}.panel{background:white;padding:20px;border-radius:12px;flex:1}
#board{display:grid;background:#d4ad70;max-width:520px;aspect-ratio:1;gap:0}.point{position:relative;display:grid;place-items:center}
.point:before{content:'';position:absolute;top:50%;height:1px;width:100%;background:#51442f}.point:after{content:'';position:absolute;left:50%;width:1px;height:100%;background:#51442f}
.stone{z-index:2;width:65%;height:65%;border-radius:50%;box-sizing:border-box}.black{background:#171b1a}.white{background:white;border:2px solid #777}.unknown{background:#c95050;color:white;text-align:center}
img{max-width:100%;max-height:530px}pre{white-space:pre-wrap;font-size:12px;max-height:260px;overflow:auto}input[type=range]{width:70%}button{padding:8px;margin:6px}small{color:#625e52}</style>
<header><h1>五子棋会话回放</h1><p>只读回放，不连接相机或机械臂。若配置标注 SIMULATED，图像与物理棋盘均为合成示意。</p>
<div id="meta"></div><button id="prev">上一步</button><button id="play">播放</button><button id="next">下一步</button><input id="slider" type="range" min="0" value="0"><span id="index"></span>
<p id="status"></p><label><input id="observed" type="checkbox"> 此记录有观测时显示观测棋盘（红色为 unknown）</label></header>
<main><section class="panel"><div id="board"></div><p id="version"></p><small>快照是已确认棋局；观测可能包含未提交棋步。行向下、列向右，索引从0开始。</small></section>
<section class="panel"><img id="image"><p id="imageNote"></p><pre id="details"></pre></section></main>
<script>const data=__PAYLOAD__;const records=data.session.records;const $=id=>document.getElementById(id);let timer=null;
$('slider').max=records.length-1;$('meta').textContent=JSON.stringify(data.session.metadata);
const initial=Number(location.hash.slice(1));if(Number.isInteger(initial)&&initial>=0&&initial<records.length)$('slider').value=initial;
function render(){const i=Number($('slider').value),r=records[i];let cp=data.session.records[0].initial_state;
for(let j=1;j<=i;j++)if(records[j].checkpoint)cp=records[j].checkpoint.state;
const obs=r.event?.observation;let cells=$('observed').checked&&obs?obs.cells:cp.cells;
$('board').style.gridTemplateColumns=`repeat(${cells.length},1fr)`;$('board').replaceChildren();
cells.forEach((row,ri)=>row.forEach((v,ci)=>{let p=document.createElement('div');p.className='point';p.title=`(${ri},${ci}) = ${v}`;
if(v){let s=document.createElement('div');s.className='stone '+(v===1?'black':v===-1?'white':'unknown');if(v===2)s.textContent='?';p.append(s)}$('board').append(p)}));
$('index').textContent=`${i+1}/${records.length}`;$('status').textContent=`${r.event?.event||r.kind} · ${r.checkpoint?.phase||''}`;
$('version').textContent=`已确认棋局版本 ${cp.version} · ${cells.length}×${cells.length} 交点`;
const path=r.kind==='image'?r.asset.path:r.event?.image?.path;const src=path?data.images[path]:null;
if(src){$('image').src=src;$('image').style.display='block';$('imageNote').textContent='当前记录关联的图像；合成会话不能视为真实采集。'}
else{$('image').removeAttribute('src');$('image').style.display='none';$('imageNote').textContent='此记录没有图像，或原图文件已缺失。'}
$('details').textContent=JSON.stringify(r,null,2)}
function step(d){$('slider').value=Math.max(0,Math.min(records.length-1,Number($('slider').value)+d));render()}
$('slider').oninput=render;$('observed').onchange=render;$('prev').onclick=()=>step(-1);$('next').onclick=()=>step(1);
$('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;$('play').textContent='播放'}else{timer=setInterval(()=>{if(Number($('slider').value)>=records.length-1){clearInterval(timer);timer=null;$('play').textContent='播放'}else step(1)},500);$('play').textContent='暂停'}};render();</script></html>'''
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(template.replace('__PAYLOAD__',payload),encoding='utf-8')
    return output
