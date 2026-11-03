"""생성된 SUMO 노면의 거리·차로 폭 비율을 그리는 검토용 배치도."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw, ImageFont
from sumolib.geomhelper import move2side

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/"env/sumo/sadang_manual"
net=ET.parse(OUT/"sadang.net.xml").getroot()
dx,dy=map(float,net.find("location").get("netOffset").split(","))
FONT="C:/Windows/Fonts/malgun.ttf"

def points(text):
    return [(p[0]-dx,p[1]-dy,*p[2:]) for p in [tuple(map(float,s.split(","))) for s in text.split()]]

def render(name,bounds,title,size):
    ratio=2
    width,height=size
    image=Image.new("RGB",(width*ratio,height*ratio),(229,235,225))
    d=ImageDraw.Draw(image)
    xmin,ymin,xmax,ymax=bounds
    margin=70
    scale=min((width-margin*2)/(xmax-xmin),(height-180)/(ymax-ymin))*ratio
    ox=(width*ratio-(xmax-xmin)*scale)/2
    oy=110*ratio+(height*ratio-180*ratio-(ymax-ymin)*scale)/2
    def xy(p): return (ox+(p[0]-xmin)*scale,oy+(ymax-p[1])*scale)
    def path(ps,color,lw):
        if len(ps)>1: d.line([xy(p) for p in ps],fill=color,width=max(1,round(lw*scale)),joint="curve")
    def dashed(ps,color,lw):
        cursor=0
        for a,b in zip(ps,ps[1:]):
            length=math.dist(a[:2],b[:2])
            t=0
            while t<length:
                chunk=min(length-t,3-(cursor%9) if cursor%9<3 else 9-(cursor%9))
                if chunk<.001: chunk=.001
                if cursor%9<3:
                    path([(a[0]+(b[0]-a[0])*t/length,a[1]+(b[1]-a[1])*t/length),
                          (a[0]+(b[0]-a[0])*(t+chunk)/length,a[1]+(b[1]-a[1])*(t+chunk)/length)],color,lw)
                t+=chunk;cursor+=chunk
    items=[]
    for e in net.findall("edge"):
        if e.get("function")=="internal": continue
        lanes=e.findall("lane")
        z=sum(p[2] if len(p)>2 else 0 for p in points(lanes[0].get("shape")))/len(points(lanes[0].get("shape")))
        items.append((z,0,e))
    for j in net.findall("junction"):
        if j.get("type")=="internal" or not j.get("shape"):continue
        z=float(j.get("z","0"));items.append((z,1,j))
    for z,kind,obj in sorted(items,key=lambda v:(round(v[0],1),v[1])):
        color=(58,62,68) if z<3 else (79,85,93)
        if kind:
            ps=points(obj.get("shape"))
            if len(ps)>2:d.polygon([xy(p) for p in ps],fill=color)
            continue
        lanes=obj.findall("lane")
        for lane in lanes:
            ps=points(lane.get("shape"));w=float(lane.get("width","3.25"))
            path(ps,(245,246,242),w+.4)
        for lane in lanes:
            ps=points(lane.get("shape"));w=float(lane.get("width","3.25"))
            path(ps,(49,91,121) if lane.get("allow")=="bus" else color,w)
        for lane in lanes[:-1]:
            ps=points(lane.get("shape"));w=float(lane.get("width","3.25"))
            dashed(move2side([p[:2] for p in ps],-w/2),(241,242,239),.16)
    font=ImageFont.truetype(FONT,round(17*ratio))
    heading=ImageFont.truetype(FONT,round(28*ratio))
    d.rectangle((0,0,width*ratio,82*ratio),fill=(247,249,245))
    d.text((28*ratio,18*ratio),title,font=heading,fill=(30,39,46))
    d.text(((width-75)*ratio,90*ratio),"N ↑",font=font,fill=(35,45,50))
    labels=[("양재IC",(5141,-1570)),("선암TG",(3530,-1845)),
            ("사당IC",(142,-645)),("사당역",(0,0)),("이수역",(58,920))]
    for label,p in labels:
        if xmin<p[0]<xmax and ymin<p[1]<ymax:
            x,y=xy(p);d.ellipse((x-4*ratio,y-4*ratio,x+4*ratio,y+4*ratio),fill=(190,107,41))
            box=d.textbbox((x+9*ratio,y-27*ratio),label,font=font)
            d.rectangle((box[0]-4*ratio,box[1]-2*ratio,box[2]+4*ratio,box[3]+2*ratio),fill=(249,250,247))
            d.text((x+9*ratio,y-27*ratio),label,font=font,fill=(32,41,48))
    if name=="seonam_lanes":
        for label,p,target in [("지상 3 → 1차로 진입 + 2차로 직진",(4370,-1900),(4430,-1980)),
                                ("중앙 본선 2차로",(4170,-2055),(4190,-1972)),
                                ("합류 후 3차로",(3880,-1870),(3880,-1945))]:
            x,y=xy(p);box=d.textbbox((x,y),label,font=font)
            d.line((x,y,*xy(target)),fill=(112,121,126),width=ratio)
            d.rectangle((box[0]-4*ratio,box[1]-3*ratio,box[2]+4*ratio,box[3]+3*ratio),fill=(249,250,247))
            d.text((x,y),label,font=font,fill=(32,41,48))
    bar=100 if xmax-xmin<3000 else 500
    x,y=35*ratio,(height-45)*ratio
    d.line((x,y,x+bar*scale,y),fill=(35,45,50),width=2*ratio)
    d.text((x,y-26*ratio),f"{bar} m",font=font,fill=(35,45,50))
    d.text((width*ratio-670*ratio,(height-40)*ratio),"차로 수·설정 폭 비율 · 고도/신호는 모형값 · 생성 도로 기준",font=font,fill=(65,74,79))
    attribution=ImageFont.truetype(FONT,13*ratio)
    d.text((width*ratio-670*ratio,(height-20)*ratio),"선형 참고: © OpenStreetMap 기여자 · 배치 대조: 카카오맵",font=attribution,fill=(75,84,88))
    image.resize(size,Image.Resampling.LANCZOS).save(OUT/f"{name}.png")

if __name__=="__main__":
    render("road_layout_overview",(-400,-2350,6000,1220),"양재 → 선암 → 사당 → 이수 | 전체 도로 배치",(2400,1500))
    render("seonam_layout",(3000,-2280,5480,-1220),"양재대로와 선암 일대 | 방향별 본선·지상도로 분리",(2200,1250))
    render("seonam_lanes",(3800,-2090,4750,-1780),"선암 진입부 | 차로 수와 연결",(1900,850))
