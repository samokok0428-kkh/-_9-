"""수작업 도로에서 차로 자체 교차/급반전/불연속 경로를 확인한다."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
from sumolib.net import readNet

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "env/sumo/sadang_manual"


def points(text):
    return [tuple(map(float, p.split(","))) for p in text.split()]


def crosses(a, b, c, d):
    def side(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    return side(a,b,c)*side(a,b,d)<-1e-6 and side(c,d,a)*side(c,d,b)<-1e-6


def audit():
    net = ET.parse(OUT / "sadang.net.xml").getroot()
    self_crossings, reversals = [], []
    for edge in net.findall("edge"):
        for lane in edge.findall("lane"):
            ps = points(lane.get("shape"))
            for i in range(len(ps)-1):
                for j in range(i+2,len(ps)-1):
                    if crosses(ps[i],ps[i+1],ps[j],ps[j+1]):
                        self_crossings.append(lane.get("id"))
            for a,b,c in zip(ps,ps[1:],ps[2:]):
                u,v=(b[0]-a[0],b[1]-a[1]),(c[0]-b[0],c[1]-b[1])
                if min(math.hypot(*u),math.hypot(*v)) < .2:
                    continue
                turn=math.degrees(math.acos(max(-1,min(1,(u[0]*v[0]+u[1]*v[1])/(math.hypot(*u)*math.hypot(*v))))))
                if turn>110:
                    reversals.append({"lane":lane.get("id"),"turn":round(turn,1)})
    connections={(c.get("from"),c.get("to")) for c in net.findall("connection")}
    routes=ET.parse(OUT / "sadang.rou.xml").getroot()
    invalid=[]
    for route in routes.findall("route"):
        es=route.get("edges").split()
        if len(set(es))!=len(es):
            invalid.append([route.get("id"),"repeated edge"])
        for a,b in zip(es,es[1:]):
            if (a,b) not in connections:
                invalid.append([route.get("id"),a,b])
    network=readNet(str(OUT/"sadang.net.xml"))
    near_crossings=[]
    es=network.getEdges()
    for i,e in enumerate(es):
        for f in es[i+1:]:
            if {e.getFromNode().getID(),e.getToNode().getID()} & {f.getFromNode().getID(),f.getToNode().getID()}:
                continue
            best=1000
            a_shape,b_shape=e.getShape3D(),f.getShape3D()
            for a,b in zip(a_shape,a_shape[1:]):
                for c,d in zip(b_shape,b_shape[1:]):
                    if not crosses(a,b,c,d):continue
                    u=(b[0]-a[0],b[1]-a[1]);v=(d[0]-c[0],d[1]-c[1]);w=(c[0]-a[0],c[1]-a[1])
                    den=u[0]*v[1]-u[1]*v[0]
                    t=(w[0]*v[1]-w[1]*v[0])/den
                    s=(w[0]*u[1]-w[1]*u[0])/den
                    best=min(best,abs(a[2]+t*(b[2]-a[2])-c[2]-s*(d[2]-c[2])))
            if best<4.5:
                near_crossings.append([e.getID(),f.getID(),round(best,2)])
    result={"self_crossings":sorted(set(self_crossings)),"sharp_reversals":reversals,
            "low_clearance_crossings":near_crossings,
            "invalid_routes":invalid,"routes":len(routes.findall("route"))}
    (OUT / "geometry_audit.json").write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf8")
    print(json.dumps(result,ensure_ascii=False))
    return result


if __name__=="__main__":
    audit()
